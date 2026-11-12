"""BluePilot: read a bool param that this build's params library may not know yet."""


def read_flag(params, key: str, default: bool = False) -> bool:
  """The manager (and the processes forked from it) keeps the params library it started with after an on-device
  rebuild, so a key added since raises UnknownKeyName. Read that as `default`, never as a crash."""
  try:
    return bool(params.get_bool(key))
  except Exception:
    return default
