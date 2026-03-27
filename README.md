# Capteurs Geolocalisés de la Qualité de l'Air

La qualité de l'air, notamment intérieur, est un enjeu de santé du quotidien. Ce projet propose un système de surveillance basé sur des capteurs géolocalisés qui remontent des informations sur les niveaux de CO2, d'humidité et de particules fines (PM1, PM2.5, PM10).

## Objectifs

- Géolocaliser le capteur
- Capturer la qualité de l'air, l'humidité et le CO2
- Remonter les valeurs sur **Home Assistant**
- Mettre en place des automatisations (ex : ouverture d'une fenêtre via servomoteur)

## Matériel

- M5Stack Basic Dev Kit (ESP32)
- Capteurs de qualité de l'air (PM1, PM2.5, PM10)
- Capteur CO2
- Capteur d'humidité
- Servomoteur
- Module de géolocalisation

## Stack technique

- **Langage** : C++ (framework Arduino)
- **Plateforme** : PlatformIO
- **Domotique** : Home Assistant (MQTT)

## Installation

### Prérequis

- [PlatformIO](https://platformio.org/install) (extension VS Code recommandée)
- Un M5Stack Basic connecté en USB

### Compilation et upload

```bash
pio run --target upload
```

### Moniteur série

```bash
pio device monitor
```

## Équipe

- [bosy0](https://github.com/bosy0)
- [RemRem-28](https://github.com/RemRem-28)
- [mariusduperrier](https://github.com/mariusduperrier)
- [alielazzouzi2005-del](https://github.com/alielazzouzi2005-del)
