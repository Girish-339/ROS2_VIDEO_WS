FROM ubuntu:22.04

WORKDIR /mediamtx

COPY mediamtx /usr/local/bin/mediamtx
COPY mediamtx.yml /mediamtx.yml

RUN chmod +x /usr/local/bin/mediamtx

EXPOSE 8554 8888 8889 10000-10005/udp

CMD ["/usr/local/bin/mediamtx", "-f", "/mediamtx.yml"]
