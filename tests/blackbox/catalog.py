"""Case inventory; phase-one fixtures remain independently importable."""
from cases import CASES as PHASE_ONE
from render_cases import CASES as PHASE_TWO

CASES = PHASE_ONE + PHASE_TWO
BY_ID = {item["id"]: item for item in CASES}
if len(BY_ID) != len(CASES):
    raise ValueError("duplicate black-box case ID")
