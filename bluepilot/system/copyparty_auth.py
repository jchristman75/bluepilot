"""BluePilot: password protection for the copyparty file server.

The portal stores a password in the CopypartyPassword param. When it is set, every copyparty volume is
restricted to one account and anonymous access gets nothing. The account lives in a mode-600 config file
under Paths.config_root() (tmpfs on the device), so the password is never on copyparty's command line.
Read once at manager start, which is why the portal says a reboot applies it.
"""
import os

from openpilot.common.hardware.hw import Paths

ACCOUNT = "bluepilot"  # copyparty logs in by password alone; the name is only an internal handle
CONF_NAME = "bp_copyparty.conf"


def _password(params) -> str:
  try:
    value = params.get("CopypartyPassword")
  except Exception:
    return ""
  if isinstance(value, bytes):
    value = value.decode("utf-8", errors="replace")
  return (value or "").strip()


def _write_conf(path: str, password: str) -> None:
  os.makedirs(os.path.dirname(path), exist_ok=True)
  fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
  with os.fdopen(fd, "w") as f:
    f.write(f"[accounts]\n  {ACCOUNT}: {password}\n")
  os.chmod(path, 0o600)


def apply_password(args: list[str], params, conf_dir: str | None = None) -> list[str]:
  """Return copyparty args restricted to the password account, or args unchanged when no password is set."""
  conf = os.path.join(conf_dir or Paths.config_root(), CONF_NAME)
  password = _password(params)
  if os.path.exists(conf):
    os.remove(conf)  # a cleared or invalid password must not leave a stale account behind
  if not password:
    return args

  # A password that can't be written as one config line fails closed: the volumes still name the account,
  # copyparty refuses to start with an undefined account, and the server stays down rather than open.
  valid = "\n" not in password and "\r" not in password
  if valid:
    _write_conf(conf, password)
  out = []
  for a in args:
    if a.startswith("-v") and a.count(":") >= 2:
      vol, perm = a.rsplit(":", 1)
      a = f"{vol}:{perm},{ACCOUNT}"  # perm for this account only; nobody else gets any access
    out.append(a)
  return out + (["-c", conf] if valid else [])
