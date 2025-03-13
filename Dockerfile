FROM rust:latest

RUN apt-get update -y && \
    apt-get install -y openssh-client \
        python3 python3-pip
    
RUN pip install maturin --break-system-packages
