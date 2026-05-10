.PHONY: \
	build_docker pull_docker \
	start_docker_debug start_docker_release stop_docker \
	enter_debug_docker enter_release_docker \
	clean_experiments_result \
	build_planner_debug build_planner_release \
	mass_tests

build_docker:
	docker compose build --progress=plain

pull_docker:
	docker compose pull

start_docker_debug:
	docker compose up -d mdp_debug

start_docker_release:
	docker compose up -d mdp_release

stop_docker:
	docker compose down

enter_debug_docker:
	docker exec -it mdp_debug /bin/bash

enter_release_docker:
	docker exec -it mdp_release /bin/bash

clean_experiments_result:
	python3 ./mass_test/clean_experiments.py

build_planner_debug:
	$(MAKE) -C ./STRRT_Planner build_debug
	$(MAKE) -C ./MSIRRT build_debug
	$(MAKE) -C ./RPMPLv2 build

build_planner_release:
	$(MAKE) -C ./STRRT_Planner build_release
	$(MAKE) -C ./MSIRRT build_release
	$(MAKE) -C ./RPMPLv2 build

mass_tests:
	python3 ./mass_test/do_mass_test.py