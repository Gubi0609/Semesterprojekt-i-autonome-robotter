#include "pico/stdlib.h"
#include <stdio.h>
#include <string>
#include <vector>

// Hjælpefunktion til at fjerne '\n' fra slutningen af en streng, hvis den er til stede.
std::string removeNewline(const std::string &str) {
    if (!str.empty() && str.back() == '\n') {
        return str.substr(0, str.size() - 1);
    }
    return str;
}

int main() {
    stdio_init_all();
    sleep_ms(2000);  // Vent et par sekunder for at sikre, at USB-seriel forbindelsen er klar

    int antalStrenge;
    char buffer[256];

    // Spørg brugeren om, hvor mange strenge der skal indtastes
    printf("Hvor mange strenge vil du indtaste? ");
    scanf("%d", &antalStrenge);
    
    // Ryd input-bufferen efter scanf (for at fjerne newline-tegnet)
    while(getchar() != '\n');  

    std::vector<std::string> strengListe;

    // Læs de angivne strenge
    for (int i = 0; i < antalStrenge; i++) {
        printf("Indtast streng %d: ", i + 1);
        if (fgets(buffer, sizeof(buffer), stdin) != NULL) {
            std::string inputStr(buffer);
            inputStr = removeNewline(inputStr); // Fjern eventuel newline
            strengListe.push_back(inputStr);
        } else {
            printf("Fejl ved læsning af streng %d.\n", i + 1);
        }
    }

    // Udskriv listen af modtagne strenge
    printf("\nDu indtastede følgende strenge:\n");
    for (size_t i = 0; i < strengListe.size(); i++) {
        printf("%zu: %s\n", i + 1, strengListe[i].c_str());
    }

    // Hold programmet kørende
    while (true) {
        tight_loop_contents();
    }

    return 0;
}
