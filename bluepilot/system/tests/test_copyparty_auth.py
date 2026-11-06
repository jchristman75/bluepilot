"""BluePilot: copyparty volumes are restricted to the portal-set password, and open without one."""
import os
import stat

from bluepilot.system.copyparty_auth import ACCOUNT, CONF_NAME, apply_password

BASE = ["-v/data/log:/swaglogs:r", "-v/data/media:/routes:r", "-v/data/models:/models:rw", "-p8080", "-z", "-q"]


class _Params:
  def __init__(self, value):
    self.value = value

  def get(self, key):
    assert key == "CopypartyPassword"
    return self.value


def test_no_password_leaves_args_open(tmp_path):
  assert apply_password(BASE, _Params(None), conf_dir=str(tmp_path)) == BASE
  assert apply_password(BASE, _Params(b""), conf_dir=str(tmp_path)) == BASE
  assert not (tmp_path / CONF_NAME).exists()


def test_password_restricts_every_volume(tmp_path):
  args = apply_password(BASE, _Params(b"p@ss: w#rd 1!"), conf_dir=str(tmp_path))
  vols = [a for a in args if a.startswith("-v")]
  assert vols == [f"{v},{ACCOUNT}" for v in BASE if v.startswith("-v")]
  assert args[-2:] == ["-c", str(tmp_path / CONF_NAME)]
  assert "p@ss: w#rd 1!" not in " ".join(args)  # never on the command line


def test_conf_is_private_and_holds_the_account(tmp_path):
  apply_password(BASE, _Params("hunter2"), conf_dir=str(tmp_path))
  conf = tmp_path / CONF_NAME
  assert stat.S_IMODE(os.stat(conf).st_mode) == 0o600
  assert conf.read_text() == f"[accounts]\n  {ACCOUNT}: hunter2\n"


def test_clearing_the_password_removes_the_account(tmp_path):
  apply_password(BASE, _Params("hunter2"), conf_dir=str(tmp_path))
  assert apply_password(BASE, _Params(""), conf_dir=str(tmp_path)) == BASE
  assert not (tmp_path / CONF_NAME).exists()


def test_multiline_password_fails_closed(tmp_path):
  # volumes still require the account but no account is defined, so copyparty won't start at all
  args = apply_password(BASE, _Params("a\n[accounts]\n  x: y"), conf_dir=str(tmp_path))
  assert [a for a in args if a.startswith("-v")] == [f"{v},{ACCOUNT}" for v in BASE if v.startswith("-v")]
  assert "-c" not in args
  assert not (tmp_path / CONF_NAME).exists()
