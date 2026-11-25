"""Where the vehicle is, and which way it faces."""
import contextlib

# Refuses a release that is not the active one (B-153). Tools that run this
# package from source alone have no runtime beside it; the refusal is a
# SystemExit, which this does not catch.
with contextlib.suppress(ModuleNotFoundError):
    import mongla_runtime  # noqa: F401
