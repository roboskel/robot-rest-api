CREATE DATABASE IF NOT EXISTS roboDB;

USE roboDB;

DROP TABLE IF EXISTS poses_in_missions;
DROP TABLE IF EXISTS missions;
DROP TABLE IF EXISTS poses;

CREATE TABLE IF NOT EXISTS poses (

    id INT AUTO_INCREMENT,
    x FLOAT NOT NULL,
    y FLOAT NOT NULL,
    stay FLOAT NOT NULL,

    PRIMARY KEY(id)

);

CREATE TABLE IF NOT EXISTS missions (

    id INT AUTO_INCREMENT,
    name VARCHAR(500) UNIQUE NOT NULL,

    PRIMARY KEY(id)

);

CREATE TABLE IF NOT EXISTS poses_in_missions (

    mid INT NOT NULL,
    pid INT NOT NULL,

    FOREIGN KEY(mid)
    REFERENCES missions(id),
    FOREIGN KEY(pid)
    REFERENCES poses(id)

);

DROP USER IF EXISTS 'roboskel'@'localhost';
DROP USER IF EXISTS 'roboskel'@'127.0.0.1';

CREATE USER IF NOT EXISTS 'roboskel'@'localhost' IDENTIFIED BY 'r0b0sk3l';
GRANT ALL PRIVILEGES ON * . * TO 'roboskel'@'localhost';

CREATE USER IF NOT EXISTS 'roboskel'@'127.0.0.1' IDENTIFIED BY 'r0b0sk3l';
GRANT ALL PRIVILEGES ON * . * TO 'roboskel'@'127.0.0.1';
