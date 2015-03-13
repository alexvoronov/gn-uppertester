package net.gcdc.uppertester;

public class CamEventIndication implements Response {
    byte messageType = 0x23;
    short camPduLength;
    @SizeFromField("camPduLength")
    byte[] camPdu;
}
