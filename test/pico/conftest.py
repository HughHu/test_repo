import pytest


def pytest_addoption(parser):
    """
    Adds command line options for specifying the serial port and loop test iteration.

    :param parser: The pytest parser object.
    """ 
    parser.addoption("--port", action="store", default="COM12", help="the serial port to specify")
    parser.addoption("--loop", action="store", default=10, type=int, help="iterate time to run the loop test")

@pytest.fixture
def port(request):
    """
    Returns the value of the --port command line option.

    :param request: The pytest request object.
    :return: The serial port specified by the --port option.
    """
    return request.config.getoption("--port") 

@pytest.fixture
def loop(request):
    """
    Returns the value of the --loop command line option.

    :param request: The pytest request object.
    :return: The number of iterations specified by the --loop option.
    """
    return request.config.getoption("--loop") 
