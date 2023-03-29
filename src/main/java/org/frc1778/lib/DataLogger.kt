package org.frc1778.lib

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.util.stream.Collectors


class DataLogger(
        /**
         * Get the name of the Context. This, appended with .csv, will give you the Filename of the
         * Context CSV file.
         */
        val name: String) {
    var timer: Timer
    private val value_suppliers: LinkedHashMap<String, () -> String>
    var started = false
    var started2 = false
    var target_file: File? = null
    var target_file2: File? = null
    var file_out: FileOutputStream? = null
    var file_out2: FileOutputStream? = null
    var start_time: Long = 0
    var usbExists = false

    init {
        timer = Timer()
        value_suppliers = LinkedHashMap()
        if (RobotBase.isSimulation()) {
            target_file = File("./$name.csv")
        } else {
            target_file = File("/home/lvuser/$name.csv")
            //Log to usb as well
            target_file2 = File("/media/sda1/$name.csv")
            usbExists = target_file2!!.parentFile.exists()
        }
        target_file!!.parentFile.mkdirs()
        add("time") { timer.get() }
    }

    /**
     * Add a Value Supplier to this context. The Value Supplier is a callback that is called whenever
     * the Context records data. This is used to gather the current value to record.
     *
     * @param supplier_name The name of the supplier. This may not contain commas.
     * @param value_supplier The supplier to get the Number value from.
     */
    fun add(supplier_name: String, value_supplier: () -> Double) {
        check(!started) { "Cannot add Suppliers after the Context has started!" }
        value_suppliers[supplier_name] = {value_supplier().toString()}
    }

    /**
     * Tick the context. This will cause a single entry to be added to the Context CSV file. This
     * should be called on a regular basis, so it is recommended that this is called either in a
     * Heartbeat Listener, or during a State Tick.
     */
    fun log() {
        if (!started) {
            // First Run
            started = true
            timer.start()
            start_time = timer.get().toLong()
            try {
                file_out = FileOutputStream(target_file)
                file_out!!.write(java.lang.String.join(",", value_suppliers.keys).toByteArray())
                file_out!!.write('\n'.code)
            } catch (e: IOException) {
                e.printStackTrace()
            }
        }
        try {
            file_out!!.write(java.lang.String.join(",", value_suppliers.values.stream()
                    .map { f: () -> String -> f() }.collect(Collectors.toList())).toByteArray())
            file_out!!.write('\n'.code)
        } catch (e: IOException) {
            e.printStackTrace()
        }
        logToUSB()
    }

    fun logToUSB() {
        if (!started2) {
            // First Run
            started2 = true
            if (usbExists) {
                try {
                    file_out2 = FileOutputStream(target_file2)
                    file_out2!!.write(java.lang.String.join(",", value_suppliers.keys).toByteArray())
                    file_out2!!.write('\n'.code)
                } catch (e: IOException) {
                    e.printStackTrace()
                    usbExists = false
                }
            }
        }
        if (usbExists && file_out2 != null) {
            try {
                file_out2!!.write(java.lang.String.join(",", value_suppliers.values.stream()
                        .map { f: () -> String -> f()}.collect(Collectors.toList())).toByteArray())
                file_out2!!.write('\n'.code)
            } catch (e: IOException) {
                e.printStackTrace()
            }
        } else {
            if (!usbExists) {
            } else if (usbExists && file_out2 == null) {
                println("file_out2 is null")
            } else if (usbExists) {
                println("Something's seriously broken right now!")
            }
        }
    }

    companion object {
        val fileNames: Array<String>
            get() = arrayOf()
    }
}