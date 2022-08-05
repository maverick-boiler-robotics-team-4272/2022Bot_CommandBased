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

    private SimpleWidget putEntry(String key, Object value){
        if(hasKey(key)){
            m_keyEntryMap.get(key).getEntry().setValue(value);
        }else{
            m_keyEntryMap.put(key, m_tab.add(key, value));
        }

        return m_keyEntryMap.get(key);
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

    public SimpleWidget putNumber(String key, double value){
        return putEntry(key, value);
    }

    public double getNumber(String key, double defaultValue){
        return getEntry(key, defaultValue).getDouble(defaultValue);
    }

    public double getNumber(String key){
        return getNumber(key, 0.0);
    }

    public SimpleWidget putBoolean(String key, boolean value){
        return putEntry(key, value);
    }

    public boolean getBoolean(String key, boolean defaultValue){
        return getEntry(key, defaultValue).getBoolean(defaultValue);
    }

    public boolean getBoolean(String key){
        return getBoolean(key, false);
    }

    public SimpleWidget putString(String key, String value){
        return putEntry(key, value);
    }

    public String getString(String key, String defaultValue){
        return getEntry(key, defaultValue).getString(defaultValue);
    }

    public String getString(String key){
        return getString(key, "");
    }

    public SimpleWidget putData(String key, Sendable data){
        return putEntry(key, data);
    }

    public SimpleWidget getWidget(String key){
        if(m_keyEntryMap.containsKey(key)){
            return m_keyEntryMap.get(key);
        } else {
            return null;
        }
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
