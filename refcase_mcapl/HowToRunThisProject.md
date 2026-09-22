# How to Run This Project

This document provides instructions on how to run this project. It contains the files for the Safety System **SS_SafeHaltController**, as specified in [DesignArchitecture.docx](https://amentumcmsgbr.sharepoint.com/:w:/r/sites/CRADLE/_layouts/15/Doc.aspx?sourcedoc=%7B684F4846-FB52-4646-A5F7-5A329DD85503%7D&file=DesignArchitecture.docx&wdLOR=c3B420EDC-D0D0-954B-A836-6CBDC1068EB4&action=default&mobileredirect=true).

This project works in conjunction with the [refcase_ws](https://github.com/everskyrube/refcase_ws) project.

## Intallation instructions

1. First, make sure you have [refcase_ws](https://github.com/everskyrube/refcase_ws) in your working directory and that Java 11 is installed and running on your machine.

2. It is recommended to structure your working directory as follows:

    ```
    refcase/
    ├── refcase_ws/
    └── refcase_mcapl/ (here is were you clone this repository)
    ```

3. Set the environment variable `AJPF_HOME` to the path of `refcase_mcapl/`, and ensure that `refcase_mcapl/bin` and `refcase_mcapl/lib/3rdparty/*` are included in your `CLASSPATH`:

    ```
    export AJPF_HOME=(refcase_mcapl/ location)

    export CLASSPATH=$AJPF_HOME/bin:$AJPF_HOME/lib/3rdparty/*
    ```

## To Run the Safety System

The safety system consists of a Gwendolen agent instantiated as a Java program, which you can run using the following command:

    java ail.mas.AIL $AJPF_HOME/src/examples/gwendolen/refcase/remote-inspection.ail


