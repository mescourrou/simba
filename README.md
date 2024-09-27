# Multi robot simulator


## Python bindings
To make python bindings, we use `PyO3`.

1) Install a virtual env and activate it:
```
python -m venv .env
source .env/bin/activate
```
2) Then, install maturin:
`pip install maturin`
3) To build the bindings, use `maturin develop`. It needs to be run in the virtualenv
4) To test that the package was built successfully, you can import it in python: `python -m turtlebot_simulator`