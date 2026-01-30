FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash ca-certificates \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY . /ws
CMD ["bash", "-lc", "echo MARTI container OK && ls -la"]
