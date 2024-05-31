#define TRIG_PIN 9
#define ECHO_PIN 10


//Arduino PWM Speed Control：
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

bool branchDetected = false;

void setup()
{
    Serial.begin(9600);

    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void loop()
{
    long duration;
    int distance;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);
    distance = duration * 0.034 / 2;

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance < 25 || branchDetected) 
    {

        if (branchDetected == false)
        {
            for (int val = 255; val > 0; val -=20)
            {
                digitalWrite(M1,LOW);
                analogWrite(E1, val);   //PWM Speed Control

                digitalWrite(M2, HIGH);
                analogWrite(E2, val);
                delay(100);
            }
        }
        branchDetected = true;

        Serial.println("Top detected");

        digitalWrite(M1,HIGH);
        analogWrite(E1, 120);   //PWM Speed Control

        digitalWrite(M2, LOW);
        analogWrite(E2, 120);
    }
    else
    {
        digitalWrite(M1,LOW);
        analogWrite(E1, 255);   //PWM Speed Control

        digitalWrite(M2, HIGH);
        analogWrite(E2, 255);
    }

    delay(500);
}
