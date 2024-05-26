#!/bin/sh

docker run --rm -v $(pwd)/slmodemd:/slmodemd -w /slmodemd --user $(id -u):$(id -g) builder make
