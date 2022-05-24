from setuptools import setup
import os

try:
    # pip >=20
    from pip._internal.network.session import PipSession
    from pip._internal.req import parse_requirements
except ImportError:
    try:
        # 10.0.0 <= pip <= 19.3.1
        from pip._internal.download import PipSession
        from pip._internal.req import parse_requirements
    except ImportError:
        # pip <= 9.0.3
        from pip.download import PipSession
        from pip.req import parse_requirements

requirements = parse_requirements(os.path.join(os.path.dirname(__file__), 'requirements.txt'), session=PipSession())

if __name__ == '__main__':
    setup(
    name='Avidrone',
    version='1.0',
    description='Autonomous Avalanche Rescue',
    author='Avidrone team 2021-2022',
    author_email='samuel.hernandez@wallawalla.edu',
    packages=['Avidrone'],
        install_requires=[str(requirement.requirement) for requirement in requirements],
    )
