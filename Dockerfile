FROM rust:latest

RUN apt-get update -y && \
    apt-get install -y openssh-client \
        python3 python3-pip
    
RUN pip install maturin matplotlib numpy pandas ipython virtualenv tree-sitter tree-sitter-rust mkdocs patchelf --break-system-packages
RUN virtualenv --system-site-packages /env