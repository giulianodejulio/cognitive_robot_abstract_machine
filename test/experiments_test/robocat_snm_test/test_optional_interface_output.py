"""Hide the optional WLAN notice on both native output streams only."""

# %% imports
import os
import shutil
import socket
import subprocess
import tempfile
from pathlib import Path

import pytest


# %% preparation stdout and stderr
@pytest.mark.parametrize('returncode', [0,9])
def test_native_output_filter_preserves_other_errors_and_exit_status(tmp_path: Path, returncode: int) -> None:
    """Captured controller CLI output and native stderr use the same narrow filter."""
    root = Path(__file__).parents[3]
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    shutil.copy(root/'scripts/prepare_robocat_controller.sh', scripts)
    package = tmp_path/'experiments/src/experiments/robocat_snm'
    package.mkdir(parents=True)
    for name in ('preparation_remote.sh','preparation_log.sed','native_navigation.py','native_preparation.py','__init__.py'):
        shutil.copy(root/'experiments/src/experiments/robocat_snm'/name, package/name)
    for name in ('__init__.py','_version.py'):
        shutil.copy(root/'experiments/src/experiments'/name, package.parent/name)
    binary=tmp_path/'bin';binary.mkdir()
    peer=binary/'ssh'
    shutil.copy(Path(__file__).with_name('dataset')/'optional_interface_peer.sh', peer)
    peer.chmod(0o755)
    planner=tmp_path/'.robocat-planner';planner.mkdir()
    with tempfile.TemporaryDirectory(prefix='robo-log-') as directory, socket.socket(socket.AF_UNIX) as connection:
        endpoint=Path(directory)/'socket'
        connection.bind(str(endpoint))
        (planner/'ssh.sock').symlink_to(endpoint)
        result=subprocess.run(['bash',str(scripts/'prepare_robocat_controller.sh'),'--check'],env={**os.environ,'PATH':str(binary)+os.pathsep+os.environ['PATH'],'ROBOCAT_PEER_EXIT':str(returncode)},capture_output=True,text=True,timeout=5)
    assert result.returncode == returncode
    assert result.stdout == 'CONTROLLER STATE: active\n'
    assert result.stderr == 'DDS error: connection lost\n'
