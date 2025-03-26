/**
 * @file I2CDevice.hpp
 * @brief Interface générique I2C pour ESP-IDF.
 * 
 * Cette classe encapsule la gestion du bus I2C (création, mutex, transactions),
 * et fournit une interface générique pour les modules utilisant un protocole I2C.
 * Elle permet une réutilisation facile dans les drivers de composants comme STUSB4500, INA226, etc.
 */

 #pragma once

 #include "driver/i2c_master.h"
 #include "freertos/semphr.h"
 #include <expected>
 #include <stdexcept>
 
 /**
  * @class I2CDevice
  * @brief Représente un périphérique I2C générique sous ESP-IDF.
  * 
  * Fournit des méthodes bas niveau pour écrire/lire des registres
  * en utilisant le protocole I2C maître d’ESP-IDF.
  */
 class I2CDevice {
 public:
     /**
      * @brief Constructeur complet avec création du bus I2C.
      * 
      * @param port Numéro de port I2C (généralement I2C_NUM_0 ou I2C_NUM_1).
      * @param sda GPIO SDA.
      * @param scl GPIO SCL.
      * @param freq Fréquence du bus I2C en Hz (par défaut : 100 kHz).
      * @param addr Adresse I2C du périphérique.
      */
     I2CDevice(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq, uint16_t addr);
 
     /**
      * @brief Constructeur avec bus I2C déjà initialisé (réutilisable).
      * 
      * @param bus Bus I2C existant (déjà initialisé via `i2c_new_master_bus`).
      * @param addr Adresse I2C du périphérique.
      * @param freq Fréquence du bus I2C (utilisé lors de l’ajout du périphérique).
      */
     I2CDevice(i2c_master_bus_handle_t bus, uint16_t addr, uint32_t freq = 100000);
 
     /**
      * @brief Destructeur.
      * 
      * Libère le périphérique I2C et supprime le bus s’il est possédé par cette instance.
      */
     virtual ~I2CDevice();
 
     /**
      * @brief Écrit une séquence d’octets à un registre du périphérique I2C.
      * 
      * @param reg Adresse du registre à écrire.
      * @param data Pointeur vers les données à envoyer.
      * @param len Taille des données à envoyer.
      * @return std::expected<void, std::runtime_error> 
      *         Un résultat vide si OK, ou une erreur explicite sinon.
      */
     std::expected<void, std::runtime_error> write(uint8_t reg, const uint8_t* data, size_t len);
 
     /**
      * @brief Lit une séquence d’octets à partir d’un registre du périphérique I2C.
      * 
      * @param reg Adresse du registre à lire.
      * @param data Pointeur vers le buffer de réception.
      * @param len Taille attendue de la lecture.
      * @return std::expected<void, std::runtime_error> 
      *         Un résultat vide si OK, ou une erreur explicite sinon.
      */
     std::expected<void, std::runtime_error> read(uint8_t reg, uint8_t* data, size_t len);
 
 protected:
     SemaphoreHandle_t mutex;                       ///< Mutex FreeRTOS pour accès I2C thread-safe.
     i2c_master_bus_handle_t bus_handle;            ///< Handle du bus I2C ESP-IDF.
     i2c_master_dev_handle_t dev_handle;            ///< Handle du périphérique I2C.
     i2c_device_config_t dev_cfg;                   ///< Configuration du périphérique I2C.
     bool owns_bus = false;                         ///< Indique si le bus a été créé par cette instance.
 };
 