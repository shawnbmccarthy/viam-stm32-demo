# simple Make file to capture some of the work needed
nanopb-setup:
	#
	# TODO: how to build without copying nanopb files to stm32 
	#       directory
	#
	git submodule update --init
	# copy nanopb .c/.h files to stm project
	cp ./nanopb/*.c ./stm32-f303re/Core/Src
	cp ./nanopb/*.h ./stm32-f303re/Core/Inc

setup-module-venv:
	python3 -m venv ./stm32-viam-module/venv
	. ./stm32-viam-module/venv/bin/activate; pip install --require-virtualenv -r ./stm32-viam-module/requirements.txt; deactivate

setup-env: nanopb-setup setup-module-venv

generate-proto-c:
	cd ./proto; . ../stm32-viam-module/venv/bin/activate; ../nanopb/generator/nanopb_generator.py ./*.proto; deactivate; cd ..
	mv ./proto/*.c ./stm32-f303re/Core/Src
	mv ./proto/*.h ./stm32-f303re/Core/Inc
	
generate-proto-py:
	. ./stm32-viam-module/venv/bin/activate; ./nanopb/generator/protoc -I=./proto --python_out=./stm32-viam-module ./proto/*.proto; deactivate
	
generate-proto: setup-env generate-proto-c generate-proto-py

# TODO: need to test out make process
stm32: generate-proto
	# setup stm32

# TODO: what else will be needed
viam-module:
	chmod 754 ./stm32-viam-module/run.sh

# TODO: what should all do
all:
	echo "all" 

clean: 
	# remove python proto files 
	rm -rf ./stm32-viam-module/*_pb2.py
	# remove virtual environments
	rm -rf ./stm32-viam-module/venv
	rm -rf ./proto/venv
	# py cache files
	rm -rf ./proto/__pycache__
	rm -rf ./stm32-viam-module/__pycache__
	# remove nanopb files from stm32 project
	rm -rf ./stm32-f303re/Core/Inc/pb_*.h
	rm -rf ./stm32-f303re/Core/Inc/pb.h
	rm -rf ./stm32-f303re/Core/Src/pb_*.c
	# remove proto files
	rm -rf ./stm32-f303re/Core/Inc/*.pb.h
	rm -rf ./stm32-f303re/Core/Src/*.pb.c
