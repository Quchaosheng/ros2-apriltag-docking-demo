from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest


CONFIG = Path(__file__).parents[1] / '.flake8'


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    return_code, errors = main_with_errors(argv=['--config', str(CONFIG)])
    assert return_code == 0, (
        f'Found {len(errors)} code style errors or warnings:\n' + '\n'.join(errors)
    )
