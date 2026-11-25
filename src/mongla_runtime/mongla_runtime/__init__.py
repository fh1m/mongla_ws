"""How a Mongla process runs: spin, parameters, launch defaults, restarts, priority."""
from .release import refuse_other_release

refuse_other_release()     # B-153: never run a release that is not the active one
