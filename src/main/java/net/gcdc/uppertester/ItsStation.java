package net.gcdc.uppertester;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import net.gcdc.geonetworking.Address;
import net.gcdc.geonetworking.Area;
import net.gcdc.geonetworking.BtpPacket;
import net.gcdc.geonetworking.BtpSocket;
import net.gcdc.geonetworking.Destination;
import net.gcdc.geonetworking.GeonetData;
import net.gcdc.geonetworking.GeonetDataListener;
import net.gcdc.geonetworking.GeonetStation;
import net.gcdc.geonetworking.LinkLayer;
import net.gcdc.geonetworking.LinkLayerUdpToEthernet;
import net.gcdc.geonetworking.LongPositionVector;
import net.gcdc.geonetworking.Optional;
import net.gcdc.geonetworking.Position;
import net.gcdc.geonetworking.PositionProvider;
import net.gcdc.geonetworking.StationConfig;
import net.gcdc.geonetworking.TrafficClass;
import net.gcdc.geonetworking.UpperProtocolType;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.threeten.bp.Instant;

import com.lexicalscope.jewel.cli.ArgumentValidationException;
import com.lexicalscope.jewel.cli.CliFactory;
import com.lexicalscope.jewel.cli.InvalidOptionSpecificationException;
import com.lexicalscope.jewel.cli.Option;

/**
 * Upper Tester Application that talks to Interface Under Test (geonetworking library) Test System.
 *
 * Test System is driven by TTCN-3 tools and communicates with Upper Tester Application via UDP.
 *
 * The main method expects that LinkLayer part (e.g. utoepy) of System Under Test is started
 * separately.
 *
 */
public class ItsStation implements AutoCloseable {
    private final static Logger logger = LoggerFactory.getLogger(ItsStation.class);

    private int defaultUpperTesterPort = 5000;

    private final DatagramSocket rcvSocket;
    private final GeonetStation station;
    private final BtpSocket btpSocket;

    private final ExecutorService executor = Executors.newCachedThreadPool();

    private final static double MICRODEGREE = 1E-6;
    private final static byte REPLY_SUCCESS = 0x01;
    private final static byte REPLY_FAILURE = 0x00;

    private final GeonetDataListener gnListener = new GeonetDataListener() {
        @Override public void onGeonetDataReceived(GeonetData indication) {
            try {
                sendReply(new GnEventIndication(indication.payload));
            } catch (IllegalArgumentException | IllegalAccessException | IOException e) {
                logger.warn("Failed to send unsolicited GN indication to the TestSystem", e);
            }
        }
    };

    private final Runnable btpListener = new Runnable() {
        @Override public void run() {
            try {
                BtpPacket btpPacket = btpSocket.receive();
                sendReply(new BtpEventIndication(btpPacket.payload()));
            } catch (InterruptedException | IllegalArgumentException | IllegalAccessException
                    | IOException e) {
                logger.warn("Failed to receive+send unsolicited BTP indication to the TestSystem",
                        e);
            }
        }
    };

    private final UpdatablePositionProvider position;

    private static class UpdatablePositionProvider implements PositionProvider {

        private Position currentPosition;

        private double speedMetersPerSecond = 0;
        private double headingDegreesFromNorth = 0;

        public UpdatablePositionProvider(Position initialPosition) {
            currentPosition = initialPosition;
        }

        public void
                move(double deltaLatDegrees, double deltaLonDegrees, final double deltaElevation) {
            currentPosition = new Position(
                    currentPosition.lattitudeDegrees() + deltaLatDegrees,
                    currentPosition.longitudeDegrees() + deltaLonDegrees);
        }

        @Override public LongPositionVector getLatestPosition() {
            Optional<Address> emptyAddress = Optional.empty();
            return new LongPositionVector(emptyAddress,
                    Instant.now(),
                    currentPosition,
                    true,
                    speedMetersPerSecond,
                    headingDegreesFromNorth);
        }

    }

    public ItsStation(StationConfig config, LinkLayer linkLayer, Position initialPosition, int udpPort) throws SocketException {
        position = new UpdatablePositionProvider(initialPosition);
        rcvSocket = new DatagramSocket(udpPort);
        station = new GeonetStation(config, linkLayer, position);
        new Thread(station).start();
        station.startBecon();
        btpSocket = BtpSocket.on(station);
        station.addGeonetDataListener(gnListener);
        executor.submit(btpListener);
    }

    private void sendReply(Response message) throws IllegalArgumentException,
            IllegalAccessException,
            IOException {
        logger.info("Sending message to TS: " + message);
        byte[] result = Parser.toBytes(message);
        DatagramPacket resPacket = new DatagramPacket(result, result.length);
        resPacket.setPort(defaultUpperTesterPort);
        rcvSocket.send(resPacket);
    }

    public void run() throws IOException, InstantiationException, IllegalAccessException {

        byte[] buffer = new byte[65535];
        DatagramPacket receivePacket = new DatagramPacket(buffer, buffer.length);

        while (true) {

            logger.debug("Waiting for packet");
            rcvSocket.receive(receivePacket);

            Object message = new Parser()
                    .parse2(Arrays.copyOfRange(buffer, 0, receivePacket.getLength()));

            // All messages in one big SWITCH-like statement:
            // Part 1: Common Upper Tester Primitives.
            if (message instanceof Initialize) {
                defaultUpperTesterPort = receivePacket.getPort();
                logger.info("Set default UDP port to {}", defaultUpperTesterPort);
                sendReply(new InitializeResult((byte) 0x01));
            } else if (message instanceof ChangePosition) {
                ChangePosition typedMessage = (ChangePosition) message;
                position.move(
                        typedMessage.deltaLatitude * 0.1 * MICRODEGREE,
                        typedMessage.deltaLongitude * 0.1 * MICRODEGREE,
                        typedMessage.deltaElevation);
                sendReply(new ChangePositionResult(REPLY_SUCCESS));
            } else if (message instanceof ChangePseudonym) {
                sendReply(new ChangePseudonymResult(REPLY_FAILURE));  // We don't have pseudonyms.
            } // ...to be continued after the comments:
              // Part 2: CAM Upper Tester Primitives (not implemented yet)
              // Part 3: DENM Upper Tester Primitives (not implemented yet)
              // Part 4: GeoNetworking Upper Tester Primitives
            else if (message instanceof GnTriggerGeoAnycast) {
                GnTriggerGeoAnycast typedMessage = (GnTriggerGeoAnycast) message;
                sendReply(new GnTriggerResult(sendAreaCast(
                        typedMessage.shape,
                        typedMessage.lifetime,
                        typedMessage.trafficClass,
                        typedMessage.latitude,
                        typedMessage.longitude,
                        typedMessage.distanceA,
                        typedMessage.distanceB,
                        typedMessage.angle,
                        typedMessage.payload,
                        true)));
            } else if (message instanceof GnTriggerGeoBroadcast) {
                GnTriggerGeoBroadcast typedMessage = (GnTriggerGeoBroadcast) message;
                sendReply(new GnTriggerResult(sendAreaCast(
                        typedMessage.shape,
                        typedMessage.lifetime,
                        typedMessage.trafficClass,
                        typedMessage.latitude,
                        typedMessage.longitude,
                        typedMessage.distanceA,
                        typedMessage.distanceB,
                        typedMessage.angle,
                        typedMessage.payload,
                        false)));
            } else if (message instanceof GnTriggerSHB) {
                GnTriggerSHB typedMessage = (GnTriggerSHB) message;
                Optional<LongPositionVector> sender = Optional.empty();
                GeonetData data = new GeonetData(
                        UpperProtocolType.BTP_B,
                        Destination.singleHop(),
                        Optional.of(TrafficClass.fromByte(typedMessage.trafficClass)),
                        sender,
                        typedMessage.payload);
                station.send(data);
                sendReply(new GnTriggerResult(REPLY_SUCCESS));
            } else if (message instanceof GnTriggerTSB) {
                sendReply(new GnTriggerResult(REPLY_FAILURE));
            } else if (message instanceof GnTriggerGeoUnicast) {
                sendReply(new GnTriggerResult(REPLY_FAILURE));
            } // ...to be continued after the comments:
              // Part 5: IPv6OverGeoNetworking Upper Tester Primitives (not supported)
              // Part 6: BTP Upper Tester Primitives
            else if (message instanceof BtpTriggerA) {
                BtpTriggerA typedMessage = (BtpTriggerA) message;
                btpSocket.send(BtpPacket.singleHopEmptyA(typedMessage.destinationPort,
                        typedMessage.sourcePort));
                sendReply(new BtpTriggerResult(REPLY_SUCCESS));
            } else if (message instanceof BtpTriggerB) {
                BtpTriggerB typedMessage = (BtpTriggerB) message;
                btpSocket.send(BtpPacket.singleHopEmptyB(typedMessage.destinationPort,
                        typedMessage.destinationPortInfo));
                sendReply(new BtpTriggerResult(REPLY_SUCCESS));
            } else {
                logger.warn("Unhandled message of type " + message.getClass());
            }
        }
    }

    private byte sendAreaCast(
            byte shape,
            short lifetime,
            byte trafficClass,
            int latE7,
            int lonE7,
            short distanceA,
            short distanceB,
            short angle,
            byte[] payload,
            boolean isAnycast) throws IOException {

        Position center = new Position(latE7 * 0.1 * MICRODEGREE, lonE7 * 0.1 * MICRODEGREE);

        Area area =
                shape == 0 ? Area.circle(center, (distanceA + distanceB) / 2.0) :
                        shape == 1 ? Area.rectangle(center, distanceA, distanceB, angle) :
                                shape == 2 ? Area.ellipse(center, distanceA, distanceB, angle) :
                                        null;
        if (area == null) {
            logger.error("Unrecognized shape id: " + shape);
            return REPLY_FAILURE;
        }

        Destination destination = isAnycast ?
                Destination.geoanycast(area).withMaxLifetimeSeconds(0.001 * lifetime) :
                Destination.geobroadcast(area).withMaxLifetimeSeconds(0.001 * lifetime);

        Optional<LongPositionVector> sender = Optional.empty();

        GeonetData data = new GeonetData(
                UpperProtocolType.BTP_B,
                destination,
                Optional.of(TrafficClass.fromByte(trafficClass)),
                sender,
                payload);

        station.send(data);

        return REPLY_SUCCESS;
    }

    @Override public void close() {
        rcvSocket.close();
        station.removeGeonetDataListener(gnListener);
        station.close();
        btpSocket.close();
    }

    public static class SocketAddressFromString {  // Public, otherwise JewelCLI can't access it!
        private final InetSocketAddress address;

        public SocketAddressFromString(final String addressStr) {
            String[] hostAndPort = addressStr.split(":");
            if (hostAndPort.length != 2) { throw new ArgumentValidationException(
                    "Expected host:port, got " + addressStr); }
            String hostname = hostAndPort[0];
            int port = Integer.parseInt(hostAndPort[1]);
            this.address = new InetSocketAddress(hostname, port);
        }

        public InetSocketAddress asInetSocketAddress() {
            return address;
        }
    }

    private static interface CliOptions {
        @Option double getLat();

        @Option double getLon();

        @Option SocketAddressFromString getRemoteAddressForUdpLinkLayer();

        @Option int getLocalPortForUdpLinkLayer();

        @Option int getUpperTesterUdpPort();

        @Option(helpRequest = true) boolean getHelp();
    }

    public static void main(final String[] args) throws InstantiationException, IllegalAccessException, IOException {
        try {
            CliOptions opts = CliFactory.parseArguments(CliOptions.class, args);
            StationConfig config = new StationConfig();
            boolean hasEthernetHeader = false;
            LinkLayer linkLayer = new LinkLayerUdpToEthernet(
                    opts.getLocalPortForUdpLinkLayer(),
                    opts.getRemoteAddressForUdpLinkLayer().asInetSocketAddress(),
                    hasEthernetHeader);
            Position position = new Position(opts.getLat(), opts.getLon());
            try (ItsStation sut = new ItsStation(config, linkLayer, position, opts.getUpperTesterUdpPort())) {
                sut.run();  // Infinite loop.
            }
        } catch (ArgumentValidationException e) {
            logger.error("Invalid CLI argument", e);
            System.exit(1);
        } catch (InvalidOptionSpecificationException e) {
            logger.error("CLI specificaton is not valid", e);
            System.exit(1);
        }
    }

}
