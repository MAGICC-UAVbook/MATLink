function MAVLink()
messageTypeIn = 1;
sequenceNumberIn = 45;
systemIDIn = 20;
componentIDIn = 5;
payloadIn = [ 10, 20, 30, 40];
commPort = 1;

[messageTypeOut, sequenceNumberOut, systemIDOut, componentIDOut, payloadOut] = MAVLink_Mex(messageTypeIn, sequenceNumberIn, systemIDIn, componentIDIn, payloadIn, commPort)
