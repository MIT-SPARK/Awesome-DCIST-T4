.PHONY: base base-nogpu dev up down shell build deploy clean help

# Default parallel jobs for colcon/cmake (override with: make build PARALLEL_JOBS=2)
PARALLEL_JOBS ?= $(shell nproc)

help: ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'

# ---- Image Builds ----

base: ## Build base image (GPU + CUDA + TensorRT)
	docker build -t dcist-base:latest -f base.Dockerfile ..

base-nogpu: ## Build base image (no GPU)
	docker build -t dcist-base:latest -f base-nogpu.Dockerfile ..

dev: ## Build dev image (requires base image)
	docker build -t dcist-dev:latest -f dev.Dockerfile ..

# ---- Dev Workflow ----

up: ## Start dev stack (dev container + Neo4j)
	docker compose up -d

down: ## Stop dev stack
	docker compose down

shell: ## Open a zsh shell in the dev container
	docker compose exec dev zsh

build: ## Run colcon build inside the dev container
	docker compose exec dev bash -c "\
		source /opt/ros/jazzy/setup.bash && \
		cd /root/dcist_ws && \
		colcon build --symlink-install --continue-on-error \
			--parallel-workers $(PARALLEL_JOBS) \
			--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"

build-sequential: ## Run colcon build sequentially (low RAM systems)
	docker compose exec dev bash -c "\
		source /opt/ros/jazzy/setup.bash && \
		cd /root/dcist_ws && \
		MAKEFLAGS='-j1' colcon build --symlink-install --continue-on-error \
			--parallel-workers 1 \
			--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"

# ---- Deployment ----

deploy: ## Build deployment image (multi-stage)
	docker build -t dcist-deploy:latest \
		--build-arg PARALLEL_JOBS=$(PARALLEL_JOBS) \
		-f deploy.Dockerfile ..

# ---- Cleanup ----

clean: ## Stop containers and remove all volumes (fresh start)
	docker compose down -v --remove-orphans

clean-build: ## Remove only build/install volumes (keep venvs and weights)
	docker volume rm -f $$(docker volume ls -q | grep dcist-build) 2>/dev/null || true
	docker volume rm -f $$(docker volume ls -q | grep dcist-install) 2>/dev/null || true
	docker volume rm -f $$(docker volume ls -q | grep dcist-log) 2>/dev/null || true
