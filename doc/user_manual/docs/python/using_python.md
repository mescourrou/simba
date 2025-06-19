# Using Python bindings

## Installation
First, download and install the wheel package on the [release page](https://gitlab.laas.fr/mescourrou/simba/-/releases).
You might want to use a virtual environment.

The built package support Python version from 3.10.

## Start the simulator
Examples of Python usage are available on [Gitlab](https://gitlab.laas.fr/mescourrou/simba/-/tree/master/examples/python).

Writing a Python program is close to writing a [Rust Plugin](../plugin/0_write_plugin.md), but some steps are simplified.

## API behind
The API behind the Python bindings uses multiple channels to communicate with the simulator threads.
Because of the GIL of Python, allowing only one thread, the Python code is executed in a single thread.
The simulator then waits that Python respond.