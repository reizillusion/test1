try:
    # generated action module (rosidl) may be installed as top-level `action`
    from action import *  # noqa: F401,F403
except Exception:
    # fallback: allow relative imports if rosidl generated into this package
    try:
        from ._order import *  # pragma: no cover
    except Exception:
        raise
