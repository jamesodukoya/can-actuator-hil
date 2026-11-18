import pytest

from sbw_can import load_database


def pytest_collection_modifyitems(items):
    """Copy @pytest.mark.requirements IDs into the JUnit XML as a 'requirements' property."""
    for item in items:
        marker = item.get_closest_marker("requirements")
        if marker is not None:
            item.user_properties.append(("requirements", ",".join(marker.args)))


@pytest.fixture(scope="session")
def database():
    return load_database()
