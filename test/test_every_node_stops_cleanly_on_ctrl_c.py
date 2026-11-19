"""Every node that spins treats Jazzy's Ctrl-C as a stop.

On Jazzy, SIGINT shuts the context down and the executor raises
`ExternalShutdownException` out of `spin()`. A main that catches only
KeyboardInterrupt exits 1 with a traceback, and the vision launch then
reports the process as died and RESTARTS it: on Kutum a normal Ctrl-C left
the detector process running past 30 s. The shared
`mongla_localization.spin.spin` handles it; a module that spins an executor
itself must handle it too.
"""
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[1] / 'src'
OWN_SPIN = re.compile(r'^\s*(?:ex|executor)\.spin\(\)', re.M)


def test_a_module_spinning_its_own_executor_handles_the_shutdown():
    spinners, missing = [], []
    for py in ROOT.glob('*/*/**/*.py'):
        if '/test/' in str(py):
            continue
        src = py.read_text()
        if OWN_SPIN.search(src):
            spinners.append(py.name)
            if 'ExternalShutdownException' not in src:
                missing.append(str(py.relative_to(ROOT)))
    assert 'spin.py' in spinners and 'detector_dual_node.py' in spinners, spinners
    assert not missing, (
        f'{missing} spin an executor and do not catch '
        f'ExternalShutdownException: on Jazzy a Ctrl-C exits 1 and the launch '
        f'restarts the process')
