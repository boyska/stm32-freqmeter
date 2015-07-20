v 20130925 2
C 44500 38800 0 0 0 title-A4.sym
C 46600 45000 1 90 0 resistor-1.sym
{
T 46200 45300 5 10 0 0 90 0 1
device=RESISTOR
T 45900 45800 5 10 1 1 0 0 1
refdes=R1
T 45900 45600 5 10 1 1 0 0 1
value=100k
T 46600 45000 5 10 0 1 0 0 1
footprint=ACY300
}
C 46600 44100 1 90 0 resistor-1.sym
{
T 46200 44400 5 10 0 0 90 0 1
device=RESISTOR
T 45900 44800 5 10 1 1 0 0 1
refdes=R2
T 45900 44600 5 10 1 1 0 0 1
value=100k
T 46600 44100 5 10 0 1 0 0 1
footprint=ACY100
}
C 46400 43800 1 0 0 gnd-1.sym
N 49000 43500 49000 46000 4
C 53200 44300 1 0 1 connector3-1.sym
{
T 51400 45200 5 10 0 0 0 6 1
device=CONNECTOR_3
T 53000 45400 5 10 1 1 0 0 1
refdes=J3
T 53200 44300 5 10 0 0 0 0 1
footprint=JUMPER3
}
N 46500 42600 47500 42600 4
C 44800 42400 1 0 0 connector4-1.sym
{
T 46600 43300 5 10 0 0 0 0 1
device=CONNECTOR_4
T 44800 43800 5 10 1 1 0 0 1
refdes=J1
T 44800 42400 5 10 0 1 0 0 1
footprint=JUMPER4
}
N 46500 42900 46500 43200 4
C 46300 45900 1 0 0 vcc-1.sym
C 48000 44400 1 0 0 dual-opamp-1.sym
{
T 48200 46700 5 10 0 0 180 8 1
device=DUAL_OPAMP
T 48100 45300 5 10 1 1 180 8 1
refdes=U1
T 48200 46300 5 10 0 0 180 8 1
footprint=DIP8
T 48200 46900 5 10 0 0 180 8 1
symversion=0.2
T 48000 44400 5 10 0 0 180 8 1
slot=1
T 48100 45600 5 10 1 1 0 0 1
model=LM393
}
C 49500 44400 1 0 0 dual-opamp-1.sym
{
T 49700 46700 5 10 0 0 0 0 1
device=DUAL_OPAMP
T 49600 45300 5 10 1 1 0 0 1
refdes=U1
T 49700 46300 5 10 0 0 0 0 1
footprint=DIP8
T 49700 46900 5 10 0 0 0 0 1
symversion=0.2
T 49500 44400 5 10 0 0 0 0 1
slot=2
}
C 50600 43900 1 90 0 resistor-1.sym
{
T 50200 44200 5 10 0 0 90 0 1
device=RESISTOR
T 50700 44600 5 10 1 1 0 0 1
refdes=R9
T 50700 44400 5 10 1 1 0 0 1
value=1k
T 50600 43900 5 10 0 1 0 0 1
footprint=ACY300
}
C 49600 43400 1 0 0 resistor-1.sym
{
T 49900 43800 5 10 0 0 0 0 1
device=RESISTOR
T 49700 43700 5 10 1 1 0 0 1
refdes=R7
T 50100 43700 5 10 1 1 0 0 1
value=1k
T 49600 43400 5 10 0 1 270 0 1
footprint=ACY300
}
N 49500 43900 50500 43900 4
N 49500 43900 49500 44600 4
N 49500 45000 49000 45000 4
N 50500 44800 51500 44800 4
C 48400 44100 1 0 0 gnd-1.sym
C 49900 44100 1 0 0 gnd-1.sym
C 51400 44200 1 0 0 gnd-1.sym
C 48300 45200 1 0 0 vcc-1.sym
C 49800 45200 1 0 0 vcc-1.sym
C 51300 45100 1 0 0 vcc-1.sym
C 53900 44100 1 0 0 gnd-1.sym
C 53800 45300 1 0 0 vcc-1.sym
C 53800 45300 1 270 0 capacitor-2.sym
{
T 54500 45100 5 10 0 0 270 0 1
device=POLARIZED_CAPACITOR
T 54300 44900 5 10 1 1 0 0 1
refdes=C4
T 54700 45100 5 10 0 0 270 0 1
symversion=0.1
T 54300 44700 5 10 1 1 0 0 1
value=100u
T 53800 45300 5 10 0 1 0 0 1
footprint=RCY100P
}
C 51400 43600 1 180 0 resistor-1.sym
{
T 51100 43200 5 10 0 0 180 0 1
device=RESISTOR
T 50600 43700 5 10 1 1 0 0 1
refdes=R8
T 51000 43700 5 10 1 1 0 0 1
value=1k
T 51400 43600 5 10 0 1 90 0 1
footprint=ACY300
}
C 49400 43200 1 0 0 gnd-1.sym
N 51500 43500 51400 43500 4
C 51300 43500 1 0 0 vcc-1.sym
C 47700 42600 1 90 0 capacitor-1.sym
{
T 47000 42800 5 10 0 0 90 0 1
device=CAPACITOR
T 46900 43300 5 10 1 1 0 0 1
refdes=C1
T 46800 42800 5 10 0 0 90 0 1
symversion=0.1
T 46900 43100 5 10 1 1 0 0 1
value=1n
T 47700 42600 5 10 0 1 180 0 1
footprint=ACY200
}
C 48000 45900 1 0 0 resistor-1.sym
{
T 48300 46300 5 10 0 0 0 0 1
device=RESISTOR
T 48100 46400 5 10 1 1 0 0 1
refdes=R3
T 48100 46200 5 10 1 1 0 0 1
value=47k
T 48000 45900 5 10 0 1 270 0 1
footprint=ACY100
}
N 47500 43500 46500 43500 4
N 48900 46000 49000 46000 4
N 48000 46000 48000 45000 4
N 49500 43500 49600 43500 4
N 50500 43900 50500 43500 4
C 47700 43500 1 90 0 capacitor-1.sym
{
T 47000 43700 5 10 0 0 90 0 1
device=CAPACITOR
T 46900 44200 5 10 1 1 0 0 1
refdes=C2
T 46800 43700 5 10 0 0 90 0 1
symversion=0.1
T 46900 44000 5 10 1 1 0 0 1
value=0.1u
T 47700 43500 5 10 0 1 180 0 1
footprint=ACY200
}
C 47700 45100 1 90 0 capacitor-1.sym
{
T 47000 45300 5 10 0 0 90 0 1
device=CAPACITOR
T 46900 45800 5 10 1 1 0 0 1
refdes=C3
T 46800 45300 5 10 0 0 90 0 1
symversion=0.1
T 46900 45600 5 10 1 1 0 0 1
value=0.1u
T 47700 45100 5 10 0 1 180 0 1
footprint=ACY200
}
C 47400 44800 1 0 0 gnd-1.sym
N 47500 46000 48000 46000 4
N 47000 44600 48000 44600 4
N 47500 44600 47500 44400 4
C 49000 43600 1 180 0 resistor-1.sym
{
T 48700 43200 5 10 0 0 180 0 1
device=RESISTOR
T 48100 43900 5 10 1 1 0 0 1
refdes=R4
T 48100 43700 5 10 1 1 0 0 1
value=100k
T 49000 43600 5 10 0 1 90 0 1
footprint=ACY100
}
N 48000 44600 48000 43500 4
N 48000 43500 48100 43500 4
N 46500 45000 47000 45000 4
N 47000 45000 47000 44600 4
C 49600 45000 1 90 0 resistor-1.sym
{
T 49200 45300 5 10 0 0 90 0 1
device=RESISTOR
T 49700 46100 5 10 1 1 0 0 1
refdes=R5
T 49700 45900 5 10 1 1 0 0 1
value=1k
T 49600 45000 5 10 0 1 0 0 1
footprint=ACY300
}
C 50600 44800 1 90 0 resistor-1.sym
{
T 50200 45100 5 10 0 0 90 0 1
device=RESISTOR
T 50700 45900 5 10 1 1 0 0 1
refdes=R6
T 50700 45700 5 10 1 1 0 0 1
value=1k
T 50600 44800 5 10 0 1 0 0 1
footprint=ACY100
}
C 49300 45900 1 0 0 vcc-1.sym
C 50300 45700 1 0 0 vcc-1.sym
C 47400 42300 1 0 0 gnd-1.sym