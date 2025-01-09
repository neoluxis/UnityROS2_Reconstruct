all:
	@if [ -n "$(PACK)" ]; then \
		echo "Building $(PACK)"; \
		colcon build --packages-select $(PACK); \
	else \
		colcon build; \
	fi

clean:
	@rm -fr build/ install/ log/

ep:
	@echo "Running ros_tcp_endpoint in screen ep"
	@screen -S ep -dm zsh "/home/neolux/workspace/Unity/ROS_Pack/ep.sh"
	@screen -ls

epat:
	@echo "Attaching to screen ep"
	@screen -r ep

epend:
	@echo "Killing screen ep"
	@screen -S ep -X quit

