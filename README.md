# Upper Tester Application for ITS CMS4 Plugtest

This is an Upper Tester Application for [geonetworking](https://github.com/alexvoronov/geonetworking) library to participate in ETSI Plugtest on Intelligent Transportation Systems - Cooperative Mobility Services Event 4 ([ITS CMS4](http://www.etsi.org/news-events/events/846-plugtests-2015-itscms4)). Message set for this Upper Tester Application is defined in [ETSI TR 103 099](http://webapp.etsi.org/workprogram/Report_WorkItem.asp?WKI_ID=42425) "Architecture of conformance validation framework".

### Usage
This is an Upper Tester Application for [geonetworking](https://github.com/alexvoronov/geonetworking) library. That library is not on Maven Central, so you have to install it in some repository you have access to. The easiest way is to install to your local repository (wherever Maven has it). Just download anywhere and run `mvn install`:

```
git clone https://github.com/alexvoronov/geonetworking.git
cd geonetworking
mvn install
```

Geonetworking library requires a separate Link Layer entity, you can use e.g. [utoepy](https://github.com/alexvoronov/utoepy).

First, start `utoepy`:

```
sudo python udp2eth.py 4000 en0

sudo python eth2udp.py 127.0.0.1:4001 en0
```

Then start Upper Tester Application. Example here uses geographical coordinates (57,13) and listens for Upper Tester Messages on UDP port 1600:

```
mvn exec:java -Dexec.mainClass="net.gcdc.uppertester.ItsStation" -Dexec.args="--lat 57 --lon 13 --localPortForUdpLinkLayer 4001 --remoteAddressForUdpLinkLayer 127.0.0.1:4000 --upperTesterUdpPort 1600"
```

### Acknowledgements
This implementation was partly developed within [i-GAME](http://gcdc.net/i-game) project that has received funding from the European Union's Seventh Framework Programme for research, technological development and demonstration under grant agreement no [612035](http://cordis.europa.eu/project/rcn/110506_en.html).


### License

This Upper Tester Application source code is released under the business-friendly Apache 2.0 license.
