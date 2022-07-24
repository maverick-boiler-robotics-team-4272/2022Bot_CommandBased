package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class ShuffleboardTable {
    private static Map<String, ShuffleboardTable> tableCache = new HashMap<>();

    private Map<String, SimpleWidget> m_keyEntryMap = new HashMap<>();
    private ShuffleboardTab m_tab;
    private ShuffleboardTable(String name){
        m_tab = Shuffleboard.getTab(name);
    }

    private void putEntry(String key, Object value){
        if(hasKey(key)){
            m_keyEntryMap.get(key).getEntry().setValue(value);
        }else{
            m_keyEntryMap.put(key, m_tab.add(key, value));
        }
    }

    private NetworkTableEntry getEntry(String key, Object defaultValue){
        if(!hasKey(key)){
            putEntry(key, defaultValue);
        }

        return m_keyEntryMap.get(key).getEntry();
    }

    public boolean hasKey(String key){
        return m_keyEntryMap.containsKey(key);
    }

    public void putNumber(String key, double value){
        putEntry(key, value);
    }

    public double getNumber(String key, double defaultValue){
        return getEntry(key, defaultValue).getDouble(defaultValue);
    }

    public double getNumber(String key){
        return getNumber(key, 0.0);
    }

    public void putBoolean(String key, boolean value){
        putEntry(key, value);
    }

    public boolean getBoolean(String key, boolean defaultValue){
        return getEntry(key, defaultValue).getBoolean(defaultValue);
    }

    public boolean getBoolean(String key){
        return getBoolean(key, false);
    }

    public void putString(String key, String value){
        putEntry(key, value);
    }

    public String getString(String key, String defaultValue){
        return getEntry(key, defaultValue).getString(defaultValue);
    }

    public String getString(String key){
        return getString(key, "");
    }

    public void putData(String key, Sendable data){
        putEntry(key, data);
    }

    public static ShuffleboardTable getTable(String name){
        if(tableCache.containsKey(name)){
            return tableCache.get(name);
        }else{
            ShuffleboardTable tab = new ShuffleboardTable(name);
            tableCache.put(name, tab);
            return tab;
        }
    }
}
