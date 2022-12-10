setup: requirements.txt
		python3 -m venv venv
		venv/bin/pip install --upgrade -r requirements.txt
		echo "run 'source venv/bin/activate' in the project directory"