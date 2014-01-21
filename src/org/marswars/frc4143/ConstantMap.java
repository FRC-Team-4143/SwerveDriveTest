/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.marswars.frc4143;

import com.sun.squawk.microedition.io.FileConnection;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.Enumeration;
import java.util.Hashtable;
import javax.microedition.io.Connector;

/**
 *
 * @author dquam
 */
public class ConstantMap {

    public Hashtable doubleMap;

    public ConstantMap() {
        doubleMap = new Hashtable(4);
    }
    
    public void save() {
         save("default.txt");
    }

    public void save(String fileName) {
        FileConnection fileConnection;
        DataOutputStream theFile = null;
        try {
            fileConnection = (FileConnection) Connector.open("file:///" + fileName, Connector.WRITE);
            fileConnection.create();
            theFile = fileConnection.openDataOutputStream();
            for (Enumeration e = doubleMap.keys(); e.hasMoreElements();) {
                String key = (String) e.nextElement();
                theFile.writeUTF((key + "|"));
                theFile.writeUTF(doubleMap.get(key).toString() + "\n");
                theFile.flush();
            }
            theFile.close();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
    
    public void load() {
        load("default.txt");
    }
    
    public void load(String fileName) {
        FileConnection fileConnection;
        DataInputStream theFile = null;
        try {
            fileConnection = (FileConnection)Connector.open("file:///"
                    + fileName, Connector.READ);
            theFile = fileConnection.openDataInputStream();
            while (theFile.available() > 0) {
                String key = "";
                String value = "";
                boolean isKey = true;
                String c = theFile.readUTF();
                while (!"\n".equals(c))  {
                    if ("|".equals(c)) {
                        isKey = false;
                        c = theFile.readUTF();
                        continue;
                    }
                    if (isKey) {
                        key += c;
                    } else {
                        value += c;
                    }
                    c = theFile.readUTF();
                }
                System.out.println("Loaded: " + key + " " + value);
                doubleMap.put(key, new Double(Double.parseDouble(value)));
            }
            theFile.close();
        } catch (IOException ex) {
            ex.printStackTrace();
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }
}
