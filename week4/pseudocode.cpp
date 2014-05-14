const timespan = 100;
const stepsize = 10;
int counter = 0;

// Slaap altijd iig de helft van de tijd
sleep(timespan);

while(counter < timespan) {
    counter += stepsize;

    if(counter >= timespan) {
        // Stuur een signaal & laat LED knipperen
        radio.send(); // TODO; echte functie gebruiken!
        digitalWrite(led_pin, HIGH); // laat LED branden

        // Reset de state van de machine
        counter = 0;
    }

    if(radio.available()) {// ontvang signaal
        counter = timespan - (timespan-counter)/2
    }

    sleep(stepsize);
}
