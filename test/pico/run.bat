set loop=1
set serial_port=COM12


pytest test_run.py --port=%serial_port% -loop=%loop% --junit-xml=report.xml
@REM --html=report.html -s
