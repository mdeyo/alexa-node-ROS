# alexa-node-ROS
Connecting Alexa Skills to node.js to ROS.

Current capabilities include publishing to a topic and making service calls in ROS.


![alt tag](./alexa-node-ros.png)


### Node Depedencies

After [installing Node.js](https://nodejs.org/en/), you will need to install dependencies npm:

```
cd alexa-node-ros
npm install
```

### ROS Depedencies

In addition to [installing ROS](http://wiki.ros.org/ROS/Installation) (any desktop-full, desktop, or ros-base will work), you will need to install rosbridge-server:

```
sudo apt-get install ros-<distro>-rosbridge-server
```

where distro = {hydro, indigo, jade, or kinetic...}, for example:

```
sudo apt-get install ros-kinetic-rosbridge-server
```

### Running the Example

To begin, start the rosbridge websocket with the following:

``
roslaunch rosbridge_server rosbridge_websocket.launch
``

To start the alexa-app-server, navigate into the examples directory and run the server using node:

```
cd examples/
sudo node server.js
```

### Testing the skills

If there are no errors, then you can navigate to http://localhost:8081/alexa/publish-example in your browser for the interactive Echo simulator built by [alexa-js](https://github.com/alexa-js)

You can select IntentRequest and PublishHelloIntent to prepare to send the first intent in the provided publish-example alexa-app. Pressing 'Send Request' will send the request object to your local alexa-app the same way that the Alexa Skills Kit will if a connected Echo voice interface understands the intent.

You can see the ROS interface in action by running
```
rostopic echo /alexa_msgs
```
before sending either the PublishHelloIntent or PublishGoodbyeIntent.

You can also copy-paste the schema and utterances from the simulated Echo interface into your Alexa Skill kit (Echo developer page) to ensure that they match on both ends.


### Configuring Your Web Service to Use a Self-Signed Certificate
When the Alexa service communicates with your web service, user requests and corresponding responses are transmitted over the Internet. To protect the confidentiality and integrity of this data, Alexa strictly enforces that HTTP connections are secured using SSL/TLS.


For testing purposes, you can use a self-signed SSL certificate to meet this requirement. In this case, you can create the certificate yourself, upload it to the Developer Portal when you set up a new skill, and configure your endpoint to present this certificate when it connects to Alexa. Note that this option can only be used for testing.


Note: If you are hosting your web service on an endpoint for which you already have a certificate signed by an Amazon-approved certificate authority, you do not need to create a self-signed certificate. You just need to configure your endpoint to present the signed certificate.


See the following sections to set up a self-signed certificate for testing. These steps use OpenSSL on the Linux platform.


### Create a Private Key and Self-signed Certificate for Testing.


Update the Alexa Skill Configuration with the Self-Signed Certificate.
Configure your Endpoint with the Self-signed Certificate.
Other SSL Resources.


Create a Private Key and Self-Signed Certificate for Testing
Run the following command to create a private key:

```
  openssl genrsa -out private-key.pem 2048
```

Important: Anyone in possession of your private key could masquerade as your service, so store your key in a secure location.


Use a text editor to create a configuration file in the following form and save it as a .cnf file (for instance, configuration.cnf):

```
[req]
distinguished_name = req_distinguished_name
x509_extensions = v3_req
prompt = no

[req_distinguished_name]
C = US
ST = Provide your two letter state abbreviation
L = Provide the name of the city in which you are located
O = Provide a name for your organization
CN = Provide a name for the skill

[v3_req]
keyUsage = keyEncipherment, dataEncipherment
extendedKeyUsage = serverAuth
subjectAltName = @subject_alternate_names

[subject_alternate_names]
DNS.1 = Provide your fully qualified domain name
or
IP.1 = Provide your the machine's IP address
```

Replace the following content in the configuration file with your own values:
ST: Provide your two letter state abbreviation
L: Provide the name of the city in which you are located
O: Provide a name for your organization
CN: Provide a name for the skill
DNS.1: Provide the fully qualified domain name for your endpoint


Note that you must provide the domain name for your endpoint in the DNS.1 section, so you may want to wait to create the certificate until you have this information.


See below for a completed sample configuration file.


Use the following command to generate a certificate. Specify the names you used for your private-key.pem and configuration.cnf files:

```
openssl req -new -x509 -days 365 \
            -key private-key.pem \
            -config configuration.cnf \
            -out certificate.pem
```

This produces a self-signed certificate in a file called certificate.pem.
Save the certificate .pem, private key .pem, and the configuration .cnf files in a safe place, then update the skill configuration with the certificate.


For example, a completed configuration file for a certificate looks similar to the following example:

```
[req]
distinguished_name = req_distinguished_name
x509_extensions = v3_req
prompt = no

[req_distinguished_name]
C = US
ST = WA
L = Seattle
O = My Company Name
CN = Wise Guy

[v3_req]
keyUsage = keyEncipherment, dataEncipherment
extendedKeyUsage = serverAuth
subjectAltName = @subject_alternate_names

[subject_alternate_names]
DNS.1 = wiseguy.mywebserver.com
```

Place the two generated files in the sslcert directory.


Make sure the following properties are in your config options that create the server:

```
AlexaAppServer.start({
  httpsPort: 443,
  httpsEnabled: true,
  privateKey: 'private-key.pem',
  certificate: 'cert.cer'
});
```



### Update the Alexa Skill Configuration with the Self-Signed Certificate
After creating your certificate, you need to update the configuration in the developer portal. Unlike your private key, the certificate only contains public data and can be shared with Amazon for the purposes of identifying your service. This lets Alexa confirm the validity of the public key portion of the certificate.
* Log on to the Developer Portal.
* Click Apps & Services and then Alexa. This displays a list of your existing Alexa skills.
* Find the skill to change in the list and choose Edit.
* Click SSL Certificate.
* Select the option I will upload a self-signed certificate.
* Open your certificate’s .pem file in a text editor, copy the entire contents, and paste it into the provided text box. The text pasted into the box should look similar to this:

```
----BEGIN CERTIFICATE----

Encrypted data

-----END CERTIFICATE-----
```

### Configure your Endpoint with the Self-Signed Certificate
When Alexa sends a request, your service must present your certificate. The subject alternate name in your certificate must match the domain name of your endpoint.


For example, assume your service’s endpoint is at https://wiseguy.mywebserver.com/wiseguy. In this case, your endpoint needs to present a valid certificate in which the subject alternate name is set to wiseguy.mywebserver.com. You specify this in the configuration file that you use to generate the certificate.


Configure your endpoint to present this certificate. The specifics for doing this depend on how you are hosting the web service. For example, if you use Amazon Web Services Elastic Beanstalk, you upload the certificate file using the AWS Command Line Interface.
