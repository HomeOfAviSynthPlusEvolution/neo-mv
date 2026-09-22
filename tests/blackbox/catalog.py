"""Case inventory; phase-one fixtures remain independently importable."""
from cases import CASES as PHASE_ONE
from render_cases import CASES as PHASE_TWO
from mask_cases import CASES as PHASE_THREE
from flow_cases import CASES as FLOW_CASES
from interpolation_cases import CASES as PHASE_FOUR
from depan_cases import CASES as PHASE_FIVE
from estimate_cases import CASES as PHASE_SIX
from stabilise_cases import CASES as PHASE_SEVEN

CASES = PHASE_ONE + PHASE_TWO + PHASE_THREE + FLOW_CASES + PHASE_FOUR + PHASE_FIVE + PHASE_SIX + PHASE_SEVEN
BY_ID = {item["id"]: item for item in CASES}
if len(BY_ID) != len(CASES):
    raise ValueError("duplicate black-box case ID")
