# Runs the zebra-crossing simulator web app (simulation/server.py).
# The server compiles the real Arduino sketches with make at startup, so the
# image needs a C++ compiler; Python is used only from its standard library.
FROM python:3.12-slim

RUN apt-get update \
 && apt-get install -y --no-install-recommends g++ make \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY firmware ./firmware
COPY simulation ./simulation

# Build once into the image so the container starts fast.
RUN make -C simulation

ENV HOST=0.0.0.0 \
    PORT=8080 \
    PYTHONUNBUFFERED=1
EXPOSE 8080

CMD ["python3", "simulation/server.py"]
