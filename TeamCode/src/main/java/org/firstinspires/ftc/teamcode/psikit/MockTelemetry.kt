package org.firstinspires.ftc.teamcode.psikit

import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.psilynx.psikit.core.Logger

class MockTelemetry(
    private val telemetry: Telemetry?
) : Telemetry {
    override fun addData(
        caption: String,
        format: String,
        vararg args: Any
    ): Telemetry.Item? {
        Logger.recordOutput("Telemetry/$caption", String.format(format, *args))
        return telemetry?.addData(caption, format, args)
    }

    override fun addData(
        caption: String,
        value: Any
    ): Telemetry.Item? {
        Logger.recordOutput("Telemetry/$caption", value.toString())
        return telemetry?.addData(caption, value)
    }

    override fun <T : Any> addData(
        caption: String,
        valueProducer: Func<T>
    ): Telemetry.Item? {
        Logger.recordOutput("Telemetry/$caption", valueProducer.value().toString())
        return telemetry?.addData(caption, valueProducer)
    }

    override fun <T : Any> addData(
        caption: String,
        format: String,
        valueProducer: Func<T>
    ): Telemetry.Item? {
        Logger.recordOutput("Telemetry/$caption", String.format(format, valueProducer.value()))
        return telemetry?.addData(caption, format, valueProducer)
    }

    override fun removeItem(item: Telemetry.Item): Boolean {
        return telemetry?.removeItem(item) ?: false
    }

    override fun clear() {
        telemetry?.clear()
    }

    override fun clearAll() {
        telemetry?.clearAll()
    }

    override fun addAction(action: Runnable): Any? {
        return telemetry?.addAction(action)
    }

    override fun removeAction(token: Any): Boolean {
        return telemetry?.removeAction(token) ?: false
    }

    override fun speak(text: String) {
        telemetry?.speak(text)
    }

    override fun speak(
        text: String,
        languageCode: String,
        countryCode: String
    ) {
        telemetry?.speak(text, languageCode, countryCode)
    }

    override fun update(): Boolean {
        return telemetry?.update() ?: false
    }

    override fun addLine(): Telemetry.Line? {
        return telemetry?.addLine()
    }

    override fun addLine(lineCaption: String): Telemetry.Line? {
        return telemetry?.addLine(lineCaption)
    }

    override fun removeLine(line: Telemetry.Line): Boolean {
        return telemetry?.removeLine(line) ?: false
    }

    override fun isAutoClear(): Boolean {
        return telemetry?.isAutoClear ?: false
    }

    override fun setAutoClear(autoClear: Boolean) {
        telemetry?.isAutoClear = autoClear
    }

    override fun getMsTransmissionInterval(): Int {
        return telemetry?.msTransmissionInterval ?: 0
    }

    override fun setMsTransmissionInterval(msTransmissionInterval: Int) {
        telemetry?.msTransmissionInterval = msTransmissionInterval
    }

    override fun getItemSeparator(): String? {
        return telemetry?.itemSeparator
    }

    override fun setItemSeparator(itemSeparator: String) {
        telemetry?.itemSeparator = itemSeparator
    }

    override fun getCaptionValueSeparator(): String? {
        return telemetry?.captionValueSeparator
    }

    override fun setCaptionValueSeparator(captionValueSeparator: String) {
        telemetry?.captionValueSeparator = captionValueSeparator
    }

    override fun setDisplayFormat(displayFormat: Telemetry.DisplayFormat) {
        telemetry?.setDisplayFormat(displayFormat)
    }

    override fun log(): Telemetry.Log? {
        return telemetry?.log()
    }

}