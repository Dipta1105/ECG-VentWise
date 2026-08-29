#include <HardwareSerial.h>

// ============================================================
// ESP32 DEVKIT V1 + SIM800L
// AUTOMATIC GSM DIAGNOSTIC + RECOVERY SYSTEM
// ============================================================
//
// CONNECTIONS
//
// SIM800L TXD  ---> ESP32 GPIO16 (RX2)
// SIM800L RXD  <--- ESP32 GPIO17 (TX2)
// SIM800L GND  ---> ESP32 GND
//
// SIM800L VCC  ---> EXTERNAL ~4.0V - 4.2V SUPPLY
//
// IMPORTANT:
// SIM800L requires high current during GSM transmission.
// Use a stable supply capable of handling current bursts.
//
// ============================================================


// ============================================================
// PIN CONFIGURATION
// ============================================================

#define GSM_RX 16
#define GSM_TX 17

#define GSM_BAUD 9600


// ============================================================
// PHONE NUMBER FOR FUTURE SMS TEST
// ============================================================

const char PHONE_NUMBER[] = "+91XXXXXXXXXX";


// ============================================================
// SERIAL PORT
// ============================================================

HardwareSerial GSM(2);


// ============================================================
// TIMEOUTS
// ============================================================

#define AT_TIMEOUT              5000
#define NETWORK_TIMEOUT         60000
#define OPERATOR_SCAN_TIMEOUT   180000


// ============================================================
// GLOBAL STATUS
// ============================================================

bool gsmOK = false;
bool simOK = false;
bool networkOK = false;

int signalRSSI = -1;
int registrationStatus = -1;

String detectedOperators[10];
int detectedOperatorCount = 0;


// ============================================================
// CLEAR GSM SERIAL BUFFER
// ============================================================

void clearGSM()
{
  while (GSM.available())
  {
    GSM.read();
  }
}


// ============================================================
// SEND AT COMMAND
// ============================================================

String sendAT(String command, unsigned long timeout = AT_TIMEOUT)
{
  clearGSM();

  Serial.println();
  Serial.println("------------------------------------------------");
  Serial.print("[GSM TX] ");
  Serial.println(command);
  Serial.println("------------------------------------------------");

  GSM.println(command);

  unsigned long start = millis();

  String response = "";

  while (millis() - start < timeout)
  {
    while (GSM.available())
    {
      char c = GSM.read();

      response += c;

      Serial.write(c);
    }
  }

  Serial.println();
  Serial.println("------------------------------------------------");

  return response;
}


// ============================================================
// CHECK RESPONSE FOR OK
// ============================================================

bool isOK(String response)
{
  return response.indexOf("OK") >= 0;
}


// ============================================================
// TEST GSM COMMUNICATION
// ============================================================

bool testGSM()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 1 - GSM COMMUNICATION");
  Serial.println("==============================================");

  String response = sendAT("AT", 3000);

  if (isOK(response))
  {
    Serial.println("[PASS] SIM800L COMMUNICATION OK");

    gsmOK = true;

    return true;
  }

  Serial.println("[FAIL] SIM800L NOT RESPONDING");

  gsmOK = false;

  return false;
}


// ============================================================
// MODULE INFORMATION
// ============================================================

void moduleInformation()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 2 - MODULE INFORMATION");
  Serial.println("==============================================");

  Serial.println("[INFO] Module information:");

  sendAT("ATI", 5000);

  Serial.println();
  Serial.println("[INFO] IMEI:");

  sendAT("AT+CGSN", 5000);
}


// ============================================================
// SIM CARD TEST
// ============================================================

bool checkSIM()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 3 - SIM CARD");
  Serial.println("==============================================");

  String response = sendAT("AT+CPIN?", 3000);

  if (response.indexOf("+CPIN: READY") >= 0)
  {
    Serial.println("[PASS] SIM CARD READY");

    simOK = true;

    return true;
  }

  if (response.indexOf("SIM PIN") >= 0)
  {
    Serial.println("[FAIL] SIM CARD REQUIRES PIN");
  }
  else
  {
    Serial.println("[FAIL] SIM CARD NOT READY");
  }

  simOK = false;

  return false;
}


// ============================================================
// MODEM CONFIGURATION
// ============================================================

void configureModem()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 4 - MODEM CONFIGURATION");
  Serial.println("==============================================");

  // Disable command echo
  Serial.println("[CONFIG] Disabling echo...");
  sendAT("ATE0", 3000);

  // Full functionality
  Serial.println("[CONFIG] Setting full functionality...");
  sendAT("AT+CFUN=1", 5000);

  // Detailed GSM registration
  Serial.println("[CONFIG] Enabling detailed CREG...");
  sendAT("AT+CREG=2", 3000);

  // Detailed GPRS registration
  Serial.println("[CONFIG] Enabling detailed CGREG...");
  sendAT("AT+CGREG=2", 3000);

  // SMS text mode
  Serial.println("[CONFIG] SMS text mode...");
  sendAT("AT+CMGF=1", 3000);

  // GSM character set
  Serial.println("[CONFIG] GSM character set...");
  sendAT("AT+CSCS=\"GSM\"", 3000);

  Serial.println("[CONFIG] Modem configuration complete.");
}


// ============================================================
// SIGNAL STRENGTH
// ============================================================

void checkSignal()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 5 - SIGNAL STRENGTH");
  Serial.println("==============================================");

  String response = sendAT("AT+CSQ", 3000);

  int position = response.indexOf("+CSQ:");

  if (position < 0)
  {
    Serial.println("[FAIL] Unable to read signal strength.");

    signalRSSI = -1;

    return;
  }

  String data = response.substring(position + 5);

  int comma = data.indexOf(',');

  if (comma < 0)
  {
    Serial.println("[FAIL] Invalid CSQ response.");

    return;
  }

  signalRSSI = data.substring(0, comma).toInt();

  Serial.print("[RSSI] ");
  Serial.println(signalRSSI);

  if (signalRSSI == 99)
  {
    Serial.println("[WARN] SIGNAL UNKNOWN");
  }
  else if (signalRSSI <= 9)
  {
    Serial.println("[WARN] VERY WEAK SIGNAL");
  }
  else if (signalRSSI <= 14)
  {
    Serial.println("[WARN] WEAK SIGNAL");
  }
  else if (signalRSSI <= 20)
  {
    Serial.println("[PASS] USABLE SIGNAL");
  }
  else if (signalRSSI <= 31)
  {
    Serial.println("[PASS] STRONG SIGNAL");
  }
}


// ============================================================
// CURRENT GSM BAND
// ============================================================

void checkCurrentBand()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 6 - CURRENT GSM BAND");
  Serial.println("==============================================");

  sendAT("AT+CBAND?", 5000);
}


// ============================================================
// SUPPORTED GSM BANDS
// ============================================================

String getSupportedBands()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 7 - SUPPORTED GSM BANDS");
  Serial.println("==============================================");

  return sendAT("AT+CBAND=?", 5000);
}


// ============================================================
// AUTOMATIC BAND CONFIGURATION
// ============================================================
//
// IMPORTANT:
// SIM800L requires the band value as a quoted string:
//
// AT+CBAND="ALL_BAND"
//
// NOT:
//
// AT+CBAND=ALL_BAND
//
// ============================================================

bool configureBestBand()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 8 - AUTOMATIC BAND CONFIGURATION");
  Serial.println("==============================================");

  // ----------------------------------------------------------
  // Read supported bands
  // ----------------------------------------------------------

  Serial.println("[BAND] Checking supported bands...");

  String supported = getSupportedBands();

  // ----------------------------------------------------------
  // Read current configuration
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("[BAND] Current band configuration:");

  sendAT("AT+CBAND?", 5000);


  // ----------------------------------------------------------
  // TRY ALL_BAND
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("[BAND] Trying ALL_BAND...");

  String response =
    sendAT("AT+CBAND=\"ALL_BAND\"", 5000);

  if (isOK(response))
  {
    Serial.println("[PASS] ALL_BAND ACCEPTED");

    delay(3000);

    Serial.println("[VERIFY] Band after configuration:");

    sendAT("AT+CBAND?", 5000);

    return true;
  }

  Serial.println("[WARN] ALL_BAND rejected.");


  // ----------------------------------------------------------
  // TRY EGSM + DCS
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("[BAND] Trying EGSM_DCS_MODE...");

  response =
    sendAT("AT+CBAND=\"EGSM_DCS_MODE\"", 5000);

  if (isOK(response))
  {
    Serial.println("[PASS] EGSM_DCS_MODE ACCEPTED");

    delay(3000);

    sendAT("AT+CBAND?", 5000);

    return true;
  }

  Serial.println("[WARN] EGSM_DCS_MODE rejected.");


  // ----------------------------------------------------------
  // TRY EGSM
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("[BAND] Trying EGSM_MODE...");

  response =
    sendAT("AT+CBAND=\"EGSM_MODE\"", 5000);

  if (isOK(response))
  {
    Serial.println("[PASS] EGSM_MODE ACCEPTED");

    delay(3000);

    sendAT("AT+CBAND?", 5000);

    return true;
  }

  Serial.println("[WARN] EGSM_MODE rejected.");


  // ----------------------------------------------------------
  // TRY DCS
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("[BAND] Trying DCS_MODE...");

  response =
    sendAT("AT+CBAND=\"DCS_MODE\"", 5000);

  if (isOK(response))
  {
    Serial.println("[PASS] DCS_MODE ACCEPTED");

    delay(3000);

    sendAT("AT+CBAND?", 5000);

    return true;
  }

  Serial.println("[WARN] DCS_MODE rejected.");


  // ----------------------------------------------------------
  // NO BAND ACCEPTED
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("[FAIL] NO BAND CONFIGURATION ACCEPTED");

  return false;
}


// ============================================================
// READ NETWORK REGISTRATION
// ============================================================

int readRegistration()
{
  String response =
    sendAT("AT+CREG?", 3000);

  int position =
    response.indexOf("+CREG:");

  if (position < 0)
  {
    registrationStatus = -1;

    return -1;
  }

  String data =
    response.substring(position + 6);

  int comma =
    data.indexOf(',');

  if (comma < 0)
  {
    registrationStatus = -1;

    return -1;
  }

  String value =
    data.substring(comma + 1);

  registrationStatus =
    value.toInt();

  return registrationStatus;
}


// ============================================================
// INTERPRET NETWORK REGISTRATION
// ============================================================

void explainRegistration(int status)
{
  Serial.println();
  Serial.println("******** NETWORK STATUS ********");

  switch (status)
  {
    case 0:

      Serial.println("NOT REGISTERED");

      break;


    case 1:

      Serial.println("REGISTERED - HOME NETWORK");

      networkOK = true;

      break;


    case 2:

      Serial.println("SEARCHING FOR NETWORK");

      break;


    case 3:

      Serial.println("REGISTRATION DENIED");

      break;


    case 4:

      Serial.println("UNKNOWN REGISTRATION STATE");

      break;


    case 5:

      Serial.println("REGISTERED - ROAMING");

      networkOK = true;

      break;


    default:

      Serial.println("UNKNOWN REGISTRATION RESPONSE");

      break;
  }

  Serial.println("********************************");
}


// ============================================================
// GPRS REGISTRATION
// ============================================================

void checkGPRS()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 9 - GPRS REGISTRATION");
  Serial.println("==============================================");

  sendAT("AT+CGREG?", 3000);
}


// ============================================================
// CURRENT OPERATOR
// ============================================================

String checkOperator()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 10 - CURRENT OPERATOR");
  Serial.println("==============================================");

  return sendAT("AT+COPS?", 5000);
}


// ============================================================
// AUTOMATIC OPERATOR SELECTION
// ============================================================

bool automaticOperatorSelection()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 11 - AUTOMATIC OPERATOR SELECTION");
  Serial.println("==============================================");

  String response =
    sendAT("AT+COPS=0", 15000);

  if (isOK(response))
  {
    Serial.println("[PASS] AUTOMATIC OPERATOR MODE ACCEPTED");

    return true;
  }

  Serial.println("[WARN] AUTOMATIC OPERATOR MODE FAILED");

  return false;
}


// ============================================================
// WAIT FOR NETWORK
// ============================================================

bool waitForNetwork(unsigned long timeout)
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" NETWORK REGISTRATION MONITOR");
  Serial.println("==============================================");

  unsigned long start =
    millis();

  while (millis() - start < timeout)
  {
    Serial.println();
    Serial.println("[NETWORK] Checking registration...");

    int status =
      readRegistration();

    explainRegistration(status);

    checkSignal();

    if (status == 1 || status == 5)
    {
      networkOK = true;

      Serial.println();
      Serial.println("****************************************");
      Serial.println("* NETWORK REGISTRATION SUCCESSFUL      *");
      Serial.println("****************************************");

      return true;
    }

    if (status == 3)
    {
      Serial.println();
      Serial.println("[WARNING] NETWORK REGISTRATION DENIED");
      Serial.println("[INFO] Continuing diagnostic process...");
    }

    delay(5000);
  }

  networkOK = false;

  Serial.println();
  Serial.println("****************************************");
  Serial.println("* NETWORK REGISTRATION TIMEOUT         *");
  Serial.println("****************************************");

  return false;
}


// ============================================================
// OPERATOR SCAN
// ============================================================

String scanOperators()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TEST 12 - OPERATOR SCAN");
  Serial.println("==============================================");

  Serial.println();
  Serial.println("[WARNING]");
  Serial.println("Operator scan can take several minutes.");
  Serial.println("Please do not reset the ESP32.");
  Serial.println();

  String response =
    sendAT(
      "AT+COPS=?",
      OPERATOR_SCAN_TIMEOUT
    );

  Serial.println();
  Serial.println("========== OPERATOR SCAN COMPLETE ==========");

  if (response.indexOf("+COPS:") >= 0)
  {
    Serial.println("[PASS] OPERATOR INFORMATION RECEIVED");
  }
  else
  {
    Serial.println("[WARN] NO OPERATOR LIST RECEIVED");
  }

  return response;
}


// ============================================================
// EXTRACT OPERATOR CODES
// ============================================================
//
// Example response:
//
// (2,"AIRTEL","AIRTEL","40445",0)
//
// Extracted operator code:
//
// 40445
//
// ============================================================

void extractOperatorCodes(String response)
{
  detectedOperatorCount = 0;

  Serial.println();
  Serial.println("==============================================");
  Serial.println(" PARSING OPERATOR SCAN");
  Serial.println("==============================================");

  int searchPosition = 0;

  while (true)
  {
    int q1 =
      response.indexOf("\"", searchPosition);

    if (q1 < 0)
      break;

    int q2 =
      response.indexOf("\"", q1 + 1);

    if (q2 < 0)
      break;

    int q3 =
      response.indexOf("\"", q2 + 1);

    if (q3 < 0)
      break;

    int q4 =
      response.indexOf("\"", q3 + 1);

    if (q4 < 0)
      break;

    int q5 =
      response.indexOf("\"", q4 + 1);

    if (q5 < 0)
      break;

    int q6 =
      response.indexOf("\"", q5 + 1);

    if (q6 < 0)
      break;

    String operatorName =
      response.substring(q1 + 1, q2);

    String shortName =
      response.substring(q3 + 1, q4);

    String operatorCode =
      response.substring(q5 + 1, q6);


    // --------------------------------------------------------
    // Check numeric operator code
    // --------------------------------------------------------

    bool numeric = true;

    if (
      operatorCode.length() < 5 ||
      operatorCode.length() > 6
    )
    {
      numeric = false;
    }

    if (numeric)
    {
      for (
        int i = 0;
        i < operatorCode.length();
        i++
      )
      {
        if (!isDigit(operatorCode[i]))
        {
          numeric = false;
          break;
        }
      }
    }


    // --------------------------------------------------------
    // Store unique operator
    // --------------------------------------------------------

    if (numeric)
    {
      bool duplicate = false;

      for (
        int i = 0;
        i < detectedOperatorCount;
        i++
      )
      {
        if (
          detectedOperators[i] ==
          operatorCode
        )
        {
          duplicate = true;
        }
      }


      if (
        !duplicate &&
        detectedOperatorCount < 10
      )
      {
        detectedOperators[
          detectedOperatorCount
        ] = operatorCode;

        Serial.print("[FOUND] ");
        Serial.print(operatorName);
        Serial.print(" -> ");
        Serial.println(operatorCode);

        detectedOperatorCount++;
      }
    }

    searchPosition =
      q6 + 1;
  }


  Serial.println();

  Serial.print("[INFO] UNIQUE OPERATORS FOUND: ");

  Serial.println(
    detectedOperatorCount
  );
}


// ============================================================
// TRY SPECIFIC OPERATOR
// ============================================================

bool tryOperator(String operatorCode)
{
  Serial.println();
  Serial.println("==============================================");
  Serial.print(" TRYING OPERATOR ");
  Serial.println(operatorCode);
  Serial.println("==============================================");


  String command =
    "AT+COPS=1,2,\"" +
    operatorCode +
    "\"";


  String response =
    sendAT(command, 30000);


  if (!isOK(response))
  {
    Serial.println("[WARN] OPERATOR SELECTION FAILED");

    return false;
  }


  Serial.println();
  Serial.println("[INFO] OPERATOR ACCEPTED");
  Serial.println("[INFO] WAITING FOR REGISTRATION...");


  unsigned long start =
    millis();


  while (
    millis() - start <
    NETWORK_TIMEOUT
  )
  {
    int status =
      readRegistration();


    explainRegistration(status);


    if (
      status == 1 ||
      status == 5
    )
    {
      networkOK = true;

      Serial.println();
      Serial.println("****************************************");
      Serial.println("* MANUAL OPERATOR REGISTRATION SUCCESS *");
      Serial.println("****************************************");

      return true;
    }


    delay(5000);
  }


  Serial.println("[FAIL] OPERATOR DID NOT REGISTER");

  return false;
}


// ============================================================
// AUTOMATIC RECOVERY ENGINE
// ============================================================

void automaticRecovery()
{
  Serial.println();
  Serial.println();
  Serial.println("################################################");
  Serial.println("#                                              #");
  Serial.println("#       AUTOMATIC GSM RECOVERY ENGINE          #");
  Serial.println("#                                              #");
  Serial.println("################################################");


  // ==========================================================
  // RECOVERY 1
  // BAND CONFIGURATION
  // ==========================================================

  Serial.println();
  Serial.println("[RECOVERY 1] GSM BAND CONFIGURATION");

  configureBestBand();

  delay(3000);


  // ==========================================================
  // RECOVERY 2
  // AUTOMATIC OPERATOR
  // ==========================================================

  Serial.println();
  Serial.println("[RECOVERY 2] AUTOMATIC OPERATOR SELECTION");

  automaticOperatorSelection();

  delay(5000);


  // ==========================================================
  // RECOVERY 3
  // WAIT FOR REGISTRATION
  // ==========================================================

  Serial.println();
  Serial.println("[RECOVERY 3] WAITING FOR NETWORK");

  if (
    waitForNetwork(
      NETWORK_TIMEOUT
    )
  )
  {
    return;
  }


  // ==========================================================
  // RECOVERY 4
  // OPERATOR SCAN
  // ==========================================================

  Serial.println();
  Serial.println("[RECOVERY 4] AUTOMATIC OPERATOR FAILED");

  Serial.println("[RECOVERY] SCANNING NETWORKS...");


  String scanResult =
    scanOperators();


  // ==========================================================
  // RECOVERY 5
  // PARSE OPERATORS
  // ==========================================================

  extractOperatorCodes(
    scanResult
  );


  if (
    detectedOperatorCount == 0
  )
  {
    Serial.println();
    Serial.println("[FAIL] NO OPERATOR CODES DETECTED");

    return;
  }


  // ==========================================================
  // RECOVERY 6
  // TRY EACH OPERATOR
  // ==========================================================

  Serial.println();
  Serial.println("[RECOVERY 6] TRYING DETECTED OPERATORS");


  for (
    int i = 0;
    i < detectedOperatorCount;
    i++
  )
  {
    if (
      tryOperator(
        detectedOperators[i]
      )
    )
    {
      return;
    }
  }


  // ==========================================================
  // RECOVERY 7
  // RETURN TO AUTO
  // ==========================================================

  Serial.println();
  Serial.println("[RECOVERY 7] RETURNING TO AUTO MODE");

  automaticOperatorSelection();

  waitForNetwork(30000);
}


// ============================================================
// SMS DIAGNOSTIC
// ============================================================

bool sendDiagnosticSMS()
{
  if (!networkOK)
  {
    Serial.println();
    Serial.println("[SMS] BLOCKED");
    Serial.println("[SMS] NETWORK NOT REGISTERED");

    return false;
  }


  Serial.println();
  Serial.println("==============================================");
  Serial.println(" SMS DIAGNOSTIC TEST");
  Serial.println("==============================================");


  Serial.println(
    "[SMS] Sending diagnostic message..."
  );


  GSM.print("AT+CMGS=\"");
  GSM.print(PHONE_NUMBER);
  GSM.println("\"");


  delay(1500);


  String response = "";


  unsigned long start =
    millis();


  while (
    millis() - start <
    3000
  )
  {
    while (GSM.available())
    {
      char c =
        GSM.read();

      response += c;

      Serial.write(c);
    }
  }


  // ----------------------------------------------------------
  // Check SMS prompt
  // ----------------------------------------------------------

  if (
    response.indexOf(">") < 0
  )
  {
    Serial.println();
    Serial.println("[FAIL] SMS PROMPT NOT RECEIVED");

    return false;
  }


  // ----------------------------------------------------------
  // Message
  // ----------------------------------------------------------

  String message =
    "HYDROS GSM DIAGNOSTIC\n"
    "ESP32: OK\n"
    "SIM800L: OK\n"
    "NETWORK: REGISTERED\n"
    "RSSI: " +
    String(signalRSSI);


  GSM.print(message);


  // CTRL+Z
  GSM.write(26);


  // ----------------------------------------------------------
  // Wait for result
  // ----------------------------------------------------------

  start =
    millis();


  response = "";


  while (
    millis() - start <
    15000
  )
  {
    while (GSM.available())
    {
      char c =
        GSM.read();

      response += c;

      Serial.write(c);
    }
  }


  if (
    response.indexOf("OK") >= 0
  )
  {
    Serial.println();
    Serial.println("[PASS] SMS SENT SUCCESSFULLY");

    return true;
  }


  Serial.println();
  Serial.println("[FAIL] SMS FAILED");

  return false;
}


// ============================================================
// FINAL DIAGNOSTIC SUMMARY
// ============================================================

void printFinalSummary()
{
  Serial.println();
  Serial.println();
  Serial.println("################################################");
  Serial.println("#                                              #");
  Serial.println("#          GSM FINAL DIAGNOSTIC                #");
  Serial.println("#                                              #");
  Serial.println("################################################");

  Serial.println();


  Serial.print("SIM800L COMMUNICATION : ");

  if (gsmOK)
    Serial.println("PASS");
  else
    Serial.println("FAIL");


  Serial.print("SIM CARD              : ");

  if (simOK)
    Serial.println("PASS");
  else
    Serial.println("FAIL");


  Serial.print("SIGNAL RSSI           : ");

  if (signalRSSI >= 0)
    Serial.println(signalRSSI);
  else
    Serial.println("UNKNOWN");


  Serial.print("NETWORK REGISTRATION  : ");

  if (networkOK)
    Serial.println("PASS");
  else
    Serial.println("FAIL");


  Serial.println();


  if (networkOK)
  {
    Serial.println("==============================================");
    Serial.println(" GSM SYSTEM READY");
    Serial.println(" SMS CAN BE USED");
    Serial.println("==============================================");
  }
  else
  {
    Serial.println("==============================================");
    Serial.println(" GSM NETWORK REGISTRATION FAILED");
    Serial.println("==============================================");

    Serial.println();

    Serial.println("NEXT CHECKS:");

    Serial.println("1. GSM POWER SUPPLY");
    Serial.println("2. GSM ANTENNA");
    Serial.println("3. OPERATOR SCAN RESULTS");
    Serial.println("4. GSM BAND");
    Serial.println("5. IMEI");
    Serial.println("6. OPERATOR REGISTRATION");
  }

  Serial.println();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(2000);


  Serial.println();
  Serial.println();
  Serial.println("################################################");
  Serial.println("#                                              #");
  Serial.println("#      ESP32 + SIM800L AUTOMATIC DIAGNOSTIC    #");
  Serial.println("#                                              #");
  Serial.println("################################################");


  Serial.println();

  Serial.println("[BOOT] ESP32 DevKit V1");

  Serial.println("[BOOT] SIM800L");

  Serial.println("[BOOT] UART2");

  Serial.println("[BOOT] RX = GPIO16");

  Serial.println("[BOOT] TX = GPIO17");


  // ==========================================================
  // START GSM UART
  // ==========================================================

  GSM.begin(
    GSM_BAUD,
    SERIAL_8N1,
    GSM_RX,
    GSM_TX
  );


  delay(3000);


  // ==========================================================
  // DIAGNOSTIC SEQUENCE
  // ==========================================================

  if (!testGSM())
  {
    printFinalSummary();

    return;
  }


  moduleInformation();


  if (!checkSIM())
  {
    printFinalSummary();

    return;
  }


  configureModem();


  checkSignal();


  checkCurrentBand();


  getSupportedBands();


  checkGPRS();


  checkOperator();


  // ==========================================================
  // AUTOMATIC RECOVERY
  // ==========================================================

  automaticRecovery();


  // ==========================================================
  // FINAL SUMMARY
  // ==========================================================

  printFinalSummary();


  Serial.println();
  Serial.println("[SYSTEM] AUTOMATIC DIAGNOSTIC COMPLETE");

  Serial.println(
    "[SYSTEM] GSM MONITORING ACTIVE"
  );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // FORWARD UNSOLICITED GSM MESSAGES
  // ==========================================================

  while (GSM.available())
  {
    char c =
      GSM.read();

    Serial.write(c);
  }


  // ==========================================================
  // PERIODIC GSM MONITOR
  // ==========================================================

  static unsigned long lastMonitor = 0;


  if (
    millis() - lastMonitor >=
    30000
  )
  {
    lastMonitor =
      millis();


    Serial.println();
    Serial.println("==============================================");
    Serial.println("[MONITOR] PERIODIC GSM STATUS");
    Serial.println("==============================================");


    checkSignal();


    int status =
      readRegistration();


    explainRegistration(status);


    if (
      status == 1 ||
      status == 5
    )
    {
      networkOK = true;
    }
    else
    {
      networkOK = false;
    }


    checkOperator();
  }
}
