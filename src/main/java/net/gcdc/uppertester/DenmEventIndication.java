package net.gcdc.uppertester;

public class DenmEventIndication implements Response {
    byte messageType = 0x17;
    short denmPduLength;
    @SizeFromField("denmPduLength")
    byte[] denmPdu;
}
