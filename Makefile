.PHONY: build_docker pull_docker start_docker

build_docker:
	docker-compose build --progress=plain

pull_docker:
	docker compose pull

start_docker_debug:
	docker compose up -d mdp_debug

start_docker_release:
	docker compose up -d mdp_debug

stop_docker:
	docker compose down

enter_debug_docker:
	docker exec -it mdp_debug /bin/bash 
	
enter_release_docker:
	docker exec -it mdp_release /bin/bash 

clean_experiments_result:
	python3 ./mass_test/clean_experiments.py

build_planner_debug:
	cd ./STRRT_Planner
	make build_debug

	cd ../MSIRRT
	make build_debug

	cd ../RPMPLv2
	make build
	
	cd ..

mass_tests:
	python3 ./mass_test/do_mass_test.py