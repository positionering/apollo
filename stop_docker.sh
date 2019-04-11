#!/usr/bin/env bash
docker stop $(docker ps -q) || true
docker rm $(docker ps -a -q) || true
