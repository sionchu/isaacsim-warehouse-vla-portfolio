from pathlib import Path


def test_package_manifest_exists() -> None:
    package_root = Path(__file__).parents[1]
    assert (package_root / "package.xml").is_file()

