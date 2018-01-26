build-centos6-installer:
	#./scripts/create-env-file.sh
	@docker-compose -f docker-compose.yml -p scrimmage build installer
	#./scripts/extract-artifacts.sh

stop:
	@docker-compose -f docker-compose.yml -p scrimmage stop

remove-dangling-images:
	@docker rmi `docker images -q -f dangling=true`

remove-all-images:
	@docker rmi --force `docker images -q -a`
