SUBMISSION_SSH_URL = icfpc@icfpc.gnolltech.org:public/2018/
SUBMISSION_WEB_URL = https://icfpc.gnolltech.org/2018/

BUILD_NAME ?= dev
TEAM_ID ?= 83a5a58b38d74178b43b65caeef23500

PROBLEMS := $(wildcard problems/LA*.mdl)
TRACES := $(wildcard traces/LA*.nbt)

all: build

build:
	$(MAKE) -C rust build

test:
	$(MAKE) -C rust test

submission: check ${BUILD_NAME}.zip ${BUILD_NAME}.zip.hash
	scp -q -oStrictHostKeyChecking=no ${BUILD_NAME}.zip ${SUBMISSION_SSH_URL}
	curl -L \
		--data-urlencode action=submit \
		--data-urlencode privateID=$(TEAM_ID) \
		--data-urlencode submissionURL=$(SUBMISSION_WEB_URL)$(BUILD_NAME).zip \
		--data-urlencode submissionSHA=$(shell awk '{print $1;}' $(BUILD_NAME).zip.hash) \
	https://script.google.com/macros/s/AKfycbzQ7Etsj7NXCN5thGthCvApancl5vni5SFsb1UoKgZQwTzXlrH7/exec

check: problems traces
	if [ $(words $(PROBLEMS)) -ne $(words $(TRACES)) ]; then exit 1; fi

${BUILD_NAME}.zip: $(TRACES)
	-rm $@
	cd traces && \
	zip -e -P $(TEAM_ID) -r ../$@ $(subst traces/,,$?)

${BUILD_NAME}.zip.hash: ${BUILD_NAME}.zip
	shasum -a 256 $< > $@

.PHONY: all submission check test build
