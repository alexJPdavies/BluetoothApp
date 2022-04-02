package com.example.bluetoothapplication;

import android.bluetooth.BluetoothAdapter;
import androidx.appcompat.app.AppCompatActivity;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.Bundle;

import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    static final UUID mUUID = UUID.fromString("867259038102541");

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        // Create Bluetooth adapter object, then printing list of connected devices to console
        BluetoothAdapter btAdapter = BluetoothAdapter.getDefaultAdapter();
        System.out.println(btAdapter.getBondedDevices());
        //Creating Variable for MAC address of Vibration sensor, then printing device name to consol
        BluetoothDevice VibrationSensor = btAdapter.getRemoteDevice("94:3C:C6:07:26:AE");
        System.out.println(VibrationSensor.getName());
        // Create socket between two devices and print to console window
        BluetoothSocket btSocket= VibrationSensor.createInsecureRfcommSocketToServiceRecord(mUUID);
        System.out.println(btSocket);







    }
}
