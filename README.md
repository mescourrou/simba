# Multi robot simulator

## Documentation
You can compile the documentation using Cargo:
```
cargo doc --no-deps --document-private-items
```

The last documentation is available [here](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/doc/turtlebot_simulator/index.html).

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


### Waiting for PyO3
Python bindings are almost set for state estimators. However, the Python GIL locks with turtles threads. I wait for the PyO3 community to implement sub interpreters to fix that (see [Issue 576](https://github.com/PyO3/pyo3/issues/576)).