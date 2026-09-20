"""Case inventory; phase-one fixtures remain independently importable."""
from cases import CASES as PHASE_ONE
from render_cases import CASES as PHASE_TWO
from mask_cases import CASES as PHASE_THREE

CASES = PHASE_ONE + PHASE_TWO + PHASE_THREE
BY_ID = {item["id"]: item for item in CASES}
if len(BY_ID) != len(CASES):
    raise ValueError("duplicate black-box case ID")
