/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.marswars.frc4143;

import com.sun.squawk.microedition.io.FileConnection;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.Hashtable;
import javax.microedition.io.Connector;

/**
 *
 * @author dquam
 */
public class ConstantMap {

    private Hashtable m_Map;

    public ConstantMap() {
    }

    public void save(String fileName) {
        FileConnection fc;
        try {
            fc = (FileConnection) Connector.open("file:///" + fileName, Connector.WRITE);
            fc.create();
            DataOutputStream theFile = fc.openDataOutputStream();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
}
