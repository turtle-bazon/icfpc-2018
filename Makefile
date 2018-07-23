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


MAKE_PID := $(shell echo $$PPID)
JOB_FLAG := $(filter -j%, $(subst -j ,-j,$(shell ps T | grep "^\s*$(MAKE_PID).*$(MAKE)")))
JOBS     := $(subst -j,,$(JOB_FLAG))
ifeq ($(JOBS),)
JOBS := 1
endif

random_swarm:
	cd rust/scorer && cargo build --release
	cd rust/random_swarm && cargo build --release
	-find traces/ -name 'FA00*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/score-fa.sh
	-find traces/ -name 'FD00*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/score-fd.sh
	-find traces/ -name 'FR00*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/score-fr.sh
	-find traces/ -name 'FA0[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/score-fa.sh
	-find traces/ -name 'FD0[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/score-fd.sh
	-find traces/ -name 'FR0[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/score-fr.sh
	-find traces/ -name 'FA[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/score-fa.sh
	-find traces/ -name 'FD[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/score-fd.sh
	-find traces/ -name 'FR[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/score-fr.sh

validate:
	cd rust/scorer && cargo build --release
	find traces/ -name 'FA00*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/validate-fa.sh
	find traces/ -name 'FD00*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/validate-fd.sh
	find traces/ -name 'FR00*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/validate-fr.sh
	find traces/ -name 'FA0[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/validate-fa.sh
	find traces/ -name 'FD0[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/validate-fd.sh
	find traces/ -name 'FR0[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/validate-fr.sh
	find traces/ -name 'FA[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/validate-fa.sh
	find traces/ -name 'FD[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/validate-fd.sh
	find traces/ -name 'FR[1-9]*.nbt' | sort | xargs -P ${JOBS} -n 1 tools/validate-fr.sh

.PHONY: all submission check test build random_swarm validate
