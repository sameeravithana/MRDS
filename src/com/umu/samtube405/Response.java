package com.umu.samtube405;

import java.util.Map;

public interface Response
{
   public void setData(Map<String, Object> data);
   public String getPath();
   public long getTimestamp();
}
