"""Exercise manual branch/tag release selection in an isolated Git repository."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).with_name('release.py').resolve()


class ResolveTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.git('init', '-b', 'master')
        self.git('config', 'user.name', 'Release test')
        self.git('config', 'user.email', 'test@example.invalid')
        self.git('config', 'commit.gpgsign', 'false')
        self.git('config', 'core.autocrlf', 'false')
        (self.root / 'CMakeLists.txt').write_text('project(neo_mv VERSION 0.9.0)\n')
        self.git('add', 'CMakeLists.txt')
        self.git('commit', '-m', 'base')
        self.base = self.git('rev-parse', 'HEAD')
        self.git('tag', '0.9.0')

    def git(self, *args):
        return subprocess.check_output(['git', '-C', str(self.root), *args],
                                       text=True, stderr=subprocess.PIPE).strip()

    def resolve(self, kind, ref, sha=None):
        output = self.root / 'output'
        output.write_text('')
        env = dict(os.environ, GITHUB_REF_TYPE=kind, GITHUB_REF_NAME=ref,
                   GITHUB_SHA=sha or self.base,
                   GITHUB_OUTPUT=str(output), GITHUB_STEP_SUMMARY=str(self.root / 'summary'))
        result = subprocess.run([sys.executable, str(SCRIPT), 'resolve'], cwd=self.root,
                                env=env, capture_output=True, text=True)
        return result, dict(line.split('=', 1) for line in output.read_text().splitlines())

    def test_branches_only_produce_artifacts(self):
        for ref in ('master', 'feature/test', '0.9.0'):
            with self.subTest(ref=ref):
                result, output = self.resolve('branch', ref)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(output, dict(commit=self.base, kind='branch', tag='',
                                              label=f'git-{self.base[:12]}'))

    def test_version_tags_publish(self):
        for ref in ('0.9.0', 'v0.9.0', '0.9.0-rc.1'):
            with self.subTest(ref=ref):
                result, output = self.resolve('tag', ref)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(output, dict(commit=self.base, kind='tag', tag=ref, label=ref))

    def test_annotated_tag_resolves_to_commit(self):
        self.git('tag', '-a', 'v0.9.0', '-m', 'release')
        result, output = self.resolve('tag', 'v0.9.0', self.git('rev-parse', 'v0.9.0'))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(output['commit'], self.base)

    def test_invalid_or_mismatched_tag_fails(self):
        for ref in ('nightly', '1.0.0', '0.9.0\nother=value'):
            with self.subTest(ref=ref):
                result, output = self.resolve('tag', ref)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(output, {})

    def test_checkout_must_match_run_commit(self):
        self.git('commit', '--allow-empty', '-m', 'later commit')
        result, output = self.resolve('branch', 'master')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('differs from the workflow run commit', result.stderr)
        self.assertEqual(output, {})

    def test_branch_movement_does_not_change_run_commit(self):
        self.git('commit', '--allow-empty', '-m', 'branch advances')
        self.git('checkout', '--detach', self.base)
        result, output = self.resolve('branch', 'master')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(output['commit'], self.base)


if __name__ == '__main__':
    unittest.main()
