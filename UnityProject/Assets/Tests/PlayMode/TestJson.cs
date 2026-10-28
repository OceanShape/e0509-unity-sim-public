// PlayMode 테스트 보고서용 작은 JSON 직렬화 (Newtonsoft 의존 없이). 값: string/bool/number/배열/사전.
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

public static class TestJson
{
    public static string Write(object v, int indent = 0)
    {
        string pad = new string(' ', indent);
        switch (v)
        {
            case null: return "null";
            case string s: return "\"" + s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\n", "\\n") + "\"";
            case bool b: return b ? "true" : "false";
            case float f: return float.IsNaN(f) || float.IsInfinity(f) ? "null" : f.ToString("R", CultureInfo.InvariantCulture);
            case double d: return double.IsNaN(d) || double.IsInfinity(d) ? "null" : d.ToString("R", CultureInfo.InvariantCulture);
            case int i: return i.ToString(CultureInfo.InvariantCulture);
            case long l: return l.ToString(CultureInfo.InvariantCulture);
            case IDictionary dict:
            {
                var parts = new List<string>();
                foreach (DictionaryEntry kv in dict) parts.Add($"{pad}  {Write(kv.Key.ToString())}: {Write(kv.Value, indent + 2)}");
                return "{\n" + string.Join(",\n", parts) + "\n" + pad + "}";
            }
            case IEnumerable seq:
                return "[" + string.Join(", ", seq.Cast<object>().Select(x => Write(x, indent + 2))) + "]";
            default: return Write(v.ToString());
        }
    }
}
