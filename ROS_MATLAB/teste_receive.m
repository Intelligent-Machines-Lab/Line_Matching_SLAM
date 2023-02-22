rosinit
exampleHelperROSCreateSampleNetwork
sub = rossubscriber('/scan','DataFormat','struct');
pause(1);
[msg2,status,statustext] = receive(sub,10)
rosshutdown