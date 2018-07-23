
all:

build:
	cd icfpc2018_lib && cargo build
	cd scorer && cargo build
	cd random_swarm && cargo build
	cd harmonics-optimizer && cargo build

test:
	cd icfpc2018_lib && cargo test
	cd scorer && cargo test
	cd random_swarm && cargo test
	cd harmonics-optimizer && cargo test

.PHONY: all build test
